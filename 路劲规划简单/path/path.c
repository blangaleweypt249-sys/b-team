#include "path.h"

#include "chassis_main.h"
#include "dt35_pnp_link.h"
#include "imu_main.h"
#include "path_line_imu.h"
#include "path_localization.h"
#include "path_safety.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define PATH_REMOTE_TIMEOUT_MS          200U
#define PATH_NEUTRAL_COMMAND_THRESHOLD  3
#define PATH_PI                         3.14159265358979323846f
#define PATH_COMMAND_TO_MPS             \
    (PATH_PI * PATH_LINE_IMU_WHEEL_DIAMETER_M / 60.0f)
#define PATH_REACTION_TIME_S            0.080f
#define PATH_BRAKE_DECELERATION_MPS2    2.000f
#define PATH_MAP_BRAKE_DECELERATION_MPS2 1.600f
#define PATH_FRONT_BASE_DISTANCE_M      0.120f
#define PATH_LEFT_BASE_DISTANCE_M       0.100f
#define PATH_BLOCK_RELEASE_HYSTERESIS_M 0.020f
#define PATH_LASER_MIN_CM               5U
#define PATH_LASER_MAX_CM               20U
#define PATH_HARD_STOP_SPEED_MPS        0.030f
#define PATH_MAP_HARD_STOP_DISTANCE_M   0.010f
#define PATH_LASER_FILTER_SIZE          3U
#define PATH_INITIAL_SAMPLE_COUNT       3U

typedef struct
{
    int16_t vx;
    int16_t vy;
    uint8_t buttons;
    uint32_t timestamp_ms;
    bool online;
    uint32_t sequence;
} path_remote_snapshot_t;

typedef struct
{
    uint16_t sample_cm[PATH_LASER_FILTER_SIZE];
    uint8_t count;
    uint8_t write_index;
    uint32_t last_rx_ms;
    uint16_t filtered_cm;
    bool online_previous;
} path_laser_filter_t;

typedef struct
{
    uint16_t sample_cm[PATH_INITIAL_SAMPLE_COUNT];
    uint8_t count;
    uint32_t last_rx_ms;
    bool online_previous;
} path_initial_sampler_t;

/* 单写者是通信任务；1 ms 底盘任务通过 sequence seqlock 读取。 */
static volatile int16_t path_remote_vx;
static volatile int16_t path_remote_vy;
static volatile uint8_t path_remote_buttons;
static volatile uint32_t path_remote_timestamp_ms;
static volatile uint8_t path_remote_online;
static volatile uint32_t path_remote_sequence;

/* 供 LoRa 原有 Chassis_SetVelocity 语句和无小电脑拦截读取。 */
static volatile int16_t path_last_output_vx;
static volatile int16_t path_last_output_vy;
static volatile int16_t path_last_output_z;

static path_diagnostics_t path_diagnostics;
static path_laser_filter_t path_front_filter;
static path_laser_filter_t path_left_filter;
static path_initial_sampler_t path_front_initial_sampler;
static path_initial_sampler_t path_left_initial_sampler;
static uint32_t path_processed_remote_sequence;
static float path_map_origin_x_m;
static float path_map_origin_y_m;
static float path_localization_yaw_deg;
static bool path_localization_yaw_valid;
static bool path_mode_button_armed;
static bool path_yaw_was_ready;
static bool path_front_blocked;
static bool path_left_blocked;
static bool path_side_detection_done;
static bool path_mirrored_detected;

static int16_t Path_AbsCommand(int16_t value)
{
    if (value == INT16_MIN)
    {
        return INT16_MAX;
    }
    return (value < 0) ? (int16_t)-value : value;
}

static float Path_MaxFloat(float lhs, float rhs)
{
    return (lhs > rhs) ? lhs : rhs;
}

static uint16_t Path_ClampLaserCm(uint16_t distance_cm)
{
    if (distance_cm < PATH_LASER_MIN_CM)
    {
        return PATH_LASER_MIN_CM;
    }
    if (distance_cm > PATH_LASER_MAX_CM)
    {
        return PATH_LASER_MAX_CM;
    }
    return distance_cm;
}

static uint16_t Path_ConservativeLaser(const path_laser_filter_t *filter)
{
    uint16_t minimum;
    uint8_t index;

    if (filter->count == 0U)
    {
        return 0U;
    }

    minimum = filter->sample_cm[0];
    for (index = 1U; index < filter->count; index++)
    {
        if (filter->sample_cm[index] < minimum)
        {
            minimum = filter->sample_cm[index];
        }
    }
    return minimum;
}

static uint16_t Path_UpdateLaserFilter(path_laser_filter_t *filter,
                                       uint32_t last_rx_ms,
                                       uint16_t distance_cm,
                                       bool online)
{
    if (!online)
    {
        filter->online_previous = false;
        return filter->filtered_cm;
    }

    distance_cm = Path_ClampLaserCm(distance_cm);
    if (!filter->online_previous)
    {
        filter->count = 0U;
        filter->write_index = 0U;
        filter->online_previous = true;
    }
    if ((filter->count == 0U) || (last_rx_ms != filter->last_rx_ms))
    {
        filter->sample_cm[filter->write_index] = distance_cm;
        filter->write_index = (uint8_t)((filter->write_index + 1U) %
                                        PATH_LASER_FILTER_SIZE);
        if (filter->count < PATH_LASER_FILTER_SIZE)
        {
            filter->count++;
        }
        filter->last_rx_ms = last_rx_ms;
        /* 距离减小时首帧生效；增大需连续覆盖旧样本，避免过早解除。 */
        filter->filtered_cm = Path_ConservativeLaser(filter);
    }
    return filter->filtered_cm;
}

static void Path_CollectInitialSample(path_initial_sampler_t *sampler,
                                      uint16_t distance_cm,
                                      uint32_t rx_ms,
                                      bool online)
{
    if (!online)
    {
        sampler->count = 0U;
        sampler->online_previous = false;
        return;
    }

    /*
     * DT35 子板超量程时输出钳在 20 cm；20 cm 饱和视为“没有目标”，
     * 不参与初始定点。镜像侧左光面向空旷场地，因此始终无样本，
     * 常规侧只有左光真正看到西墙（< 20 cm）才允许按西墙锚定。
     */
    if (Path_ClampLaserCm(distance_cm) >= PATH_LASER_MAX_CM)
    {
        return;
    }

    if (!sampler->online_previous)
    {
        sampler->count = 0U;
        sampler->last_rx_ms = rx_ms;
        sampler->online_previous = true;
        sampler->sample_cm[sampler->count++] =
            Path_ClampLaserCm(distance_cm);
        return;
    }

    if ((sampler->count < PATH_INITIAL_SAMPLE_COUNT) &&
        (rx_ms != sampler->last_rx_ms))
    {
        sampler->last_rx_ms = rx_ms;
        sampler->sample_cm[sampler->count++] =
            Path_ClampLaserCm(distance_cm);
    }
}

static uint16_t Path_InitialMedian(const path_initial_sampler_t *sampler)
{
    uint16_t a = sampler->sample_cm[0];
    uint16_t b = sampler->sample_cm[1];
    uint16_t c = sampler->sample_cm[2];
    uint16_t swap;

    if (a > b)
    {
        swap = a;
        a = b;
        b = swap;
    }
    if (b > c)
    {
        swap = b;
        b = c;
        c = swap;
    }
    return (a > b) ? a : b;
}

static void Path_WriteRemoteMailbox(int16_t vx, int16_t vy,
                                    uint8_t buttons, uint32_t now_ms,
                                    bool online)
{
    uint32_t sequence = path_remote_sequence;

    path_remote_sequence = sequence + 1U;
    __DMB();
    path_remote_vx = vx;
    path_remote_vy = vy;
    path_remote_buttons = buttons;
    path_remote_timestamp_ms = now_ms;
    path_remote_online = online ? 1U : 0U;
    __DMB();
    path_remote_sequence = sequence + 2U;
}

static void Path_ReadRemoteMailbox(path_remote_snapshot_t *snapshot)
{
    uint32_t sequence_before;
    uint32_t sequence_after;

    do
    {
        sequence_before = path_remote_sequence;
        __DMB();
        snapshot->vx = path_remote_vx;
        snapshot->vy = path_remote_vy;
        snapshot->buttons = path_remote_buttons;
        snapshot->timestamp_ms = path_remote_timestamp_ms;
        snapshot->online = path_remote_online != 0U;
        __DMB();
        sequence_after = path_remote_sequence;
    } while (((sequence_before & 1U) != 0U) ||
             (sequence_before != sequence_after));
    snapshot->sequence = sequence_after;
}

static void Path_ProcessModeButton(const path_remote_snapshot_t *remote)
{
    const path_map_route_segment_t *route;
    bool pressed;
    uint8_t route_count;

    if (remote->sequence == path_processed_remote_sequence)
    {
        return;
    }
    path_processed_remote_sequence = remote->sequence;
    pressed = (remote->buttons & PATH_REMOTE_MODE_BUTTON_BIT) != 0U;

    if (!remote->online)
    {
        path_diagnostics.single_axis_enabled = false;
        path_mode_button_armed = false;
        return;
    }
    if (!pressed)
    {
        path_mode_button_armed = true;
        return;
    }
    if (!path_mode_button_armed)
    {
        return;
    }

    path_mode_button_armed = false;
    if (path_diagnostics.segment_index >= PATH_MAP_ROUTE_SEGMENT_COUNT)
    {
        return;
    }

    path_diagnostics.single_axis_enabled =
        !path_diagnostics.single_axis_enabled;
    path_diagnostics.manual_toggle_count++;
    route = PathMap_GetRoute(&route_count);
    if (path_diagnostics.segment_index < route_count)
    {
        path_diagnostics.active_axis =
            route[path_diagnostics.segment_index].axis;
    }
}

/*
 * 镜像侧自动锚定：前光首次扫到墙（镜像场地第一堵前墙是贴东墙的
 * 墙 B）且左光无目标时判定为镜像侧。X 采用“贴东墙对称摆放”的
 * 假定起点（与常规侧贴西墙 15 cm 间距对称），Y 用前光安装偏移
 * （0.225 m）加实测距离反算墙 B 南面位置，之后地图坐标完全跟随
 * 融合里程计。该锚定不依赖 yaw，也不依赖左 DT35。
 */
static void Path_TrySetMirroredInitialPosition(
    const path_line_imu_data_t *odometry)
{
    float front_distance_m;
    float map_y_m;

    if (!path_mirrored_detected ||
        path_diagnostics.initial_position_valid)
    {
        return;
    }

    front_distance_m = (float)path_diagnostics.front_distance_cm * 0.01f;
    map_y_m = PATH_MAP_MIRRORED_FIRST_WALL_Y_M -
              PATH_LOCALIZATION_FRONT_SENSOR_OFFSET_M -
              front_distance_m;

    PathMap_SetMirrored(true);
    path_map_origin_x_m = PATH_MAP_MIRRORED_START_X_M -
                          odometry->fused_position_x_m;
    path_map_origin_y_m = map_y_m - odometry->fused_position_y_m;
    path_diagnostics.initial_position_valid = true;
    path_diagnostics.initial_map_x_m = PATH_MAP_MIRRORED_START_X_M;
    path_diagnostics.initial_map_y_m = map_y_m;
    path_diagnostics.initial_yaw_deg = path_localization_yaw_valid ?
                                       path_localization_yaw_deg : 0.0f;
    /* 镜像侧左光无目标，前光距离成为有效初始信息。 */
    path_diagnostics.front_initial_distance_m = front_distance_m;
    path_diagnostics.left_initial_distance_m = 0.0f;
    path_diagnostics.front_wall_hit_x_m = 0.0f;
    path_diagnostics.left_wall_hit_y_m = 0.0f;
}

static void Path_TrySetInitialPosition(
    const path_line_imu_data_t *odometry)
{
    path_localization_result_t result;
    float left_distance_m;

    if (path_diagnostics.initial_position_valid ||
        !path_localization_yaw_valid ||
        (path_left_initial_sampler.count < PATH_INITIAL_SAMPLE_COUNT))
    {
        return;
    }

    left_distance_m =
        (float)Path_InitialMedian(&path_left_initial_sampler) * 0.01f;
    if (!PathLocalization_Calculate(0.0f, left_distance_m,
                                    path_localization_yaw_deg, &result))
    {
        path_diagnostics.initial_position_reject_count++;
        path_left_initial_sampler.count = 0U;
        return;
    }

    /* 只锚定一次；之后地图坐标完全跟随现有融合里程计。 */
    path_map_origin_x_m = result.map_x_m - odometry->fused_position_x_m;
    path_map_origin_y_m = result.map_y_m - odometry->fused_position_y_m;
    path_diagnostics.initial_position_valid = true;
    path_diagnostics.initial_map_x_m = result.map_x_m;
    path_diagnostics.initial_map_y_m = result.map_y_m;
    path_diagnostics.initial_yaw_deg = path_localization_yaw_deg;
    path_diagnostics.front_initial_distance_m = 0.0f;
    path_diagnostics.left_initial_distance_m = left_distance_m;
    path_diagnostics.front_wall_hit_x_m = 0.0f;
    path_diagnostics.left_wall_hit_y_m = result.left_wall_hit_y_m;
}

static bool Path_UpdateOdometryAndRoute(void)
{
    path_line_imu_data_t odometry;
    const path_map_route_segment_t *route;
    uint8_t route_count;
    bool automatic_stop = false;

    path_diagnostics.odometry_valid =
        PathLineImu_GetData(&odometry) &&
        (odometry.imu_solution_valid || odometry.encoder_solution_valid);
    if (!path_diagnostics.odometry_valid)
    {
        return false;
    }

    path_diagnostics.encoder_velocity_x_mps =
        odometry.encoder_solution_valid ?
        odometry.encoder_body_velocity_x_mps :
        odometry.fused_velocity_x_mps;
    path_diagnostics.encoder_velocity_y_mps =
        odometry.encoder_solution_valid ?
        odometry.encoder_body_velocity_y_mps :
        odometry.fused_velocity_y_mps;

    Path_TrySetMirroredInitialPosition(&odometry);
    Path_TrySetInitialPosition(&odometry);
    if (!path_diagnostics.initial_position_valid)
    {
        return false;
    }

    path_diagnostics.map_x_m = path_map_origin_x_m +
                               odometry.fused_position_x_m;
    path_diagnostics.map_y_m = path_map_origin_y_m +
                               odometry.fused_position_y_m;

    route = PathMap_GetRoute(&route_count);
    while ((path_diagnostics.segment_index < route_count) &&
           PathMap_SegmentReached(path_diagnostics.segment_index,
                                  path_diagnostics.map_x_m,
                                  path_diagnostics.map_y_m))
    {
        path_diagnostics.segment_index++;
        if (path_diagnostics.single_axis_enabled)
        {
            path_diagnostics.single_axis_enabled = false;
            path_diagnostics.neutral_rearm_required = true;
            path_diagnostics.automatic_cancel_count++;
            automatic_stop = true;
        }
    }

    path_diagnostics.route_complete =
        path_diagnostics.segment_index >= route_count;
    if (!path_diagnostics.route_complete)
    {
        path_diagnostics.active_axis =
            route[path_diagnostics.segment_index].axis;
    }
    return automatic_stop;
}

static void Path_UpdateYawZeroLock(void)
{
    imu_data_t imu;
    bool ready = ImuMain_GetData(&imu) &&
                 (imu.state == IMU_STATE_READY) && imu.online &&
                 imu.yaw_valid && !isnan(imu.yaw_deg) &&
                 !isinf(imu.yaw_deg);

    if (ready)
    {
        /* 初始定点使用锁零前这一周期实际测得的 yaw。 */
        path_localization_yaw_deg = imu.yaw_deg;
        path_localization_yaw_valid = true;
    }
    else
    {
        path_localization_yaw_valid = false;
    }

    if (ready && (!path_yaw_was_ready || !imu.yaw_hold_enabled))
    {
        ImuMain_EnableYawHold(true);
        if (ImuMain_SetTargetYaw(PATH_MAP_LOCK_YAW_DEG) != HAL_OK)
        {
            ready = false;
        }
    }
    path_yaw_was_ready = ready;
    path_diagnostics.yaw_zero_lock_ready = ready;
}

static void Path_UpdateLaserData(void)
{
    uint32_t front_last_rx;
    uint32_t left_last_rx;
    uint32_t primask;
    uint16_t front_cm;
    uint16_t left_cm;
    bool front_online;
    bool left_online;

    /* 数据由 UART 中断逐字段更新，短临界区避免拼出跨帧快照。 */
    primask = __get_PRIMASK();
    __disable_irq();
    front_last_rx = dt35_link[SENSOR_LINK_F_INDEX].last_rx_ms;
    left_last_rx = dt35_link[SENSOR_LINK_L_B_INDEX].last_rx_ms;
    front_cm = dt35_link[SENSOR_LINK_F_INDEX].distance_cm;
    left_cm = dt35_link[SENSOR_LINK_L_B_INDEX].distance_cm;
    front_online = dt35_link[SENSOR_LINK_F_INDEX].online != 0U;
    left_online = dt35_link[SENSOR_LINK_L_B_INDEX].online != 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }

    path_diagnostics.front_laser_online = front_online;
    path_diagnostics.left_laser_online = left_online;
    if (!path_diagnostics.initial_position_valid)
    {
        /* 初始 Y 固定；前 DT35 只保留运行期 +Y 动态安全职责。 */
        Path_CollectInitialSample(&path_left_initial_sampler, left_cm,
                                  left_last_rx, left_online);
    }

    /*
     * 对抗镜像侧自动识别：前光首次扫到墙（读数降到 20 cm 饱和线
     * 以下）时，检查左光是否看到目标。左光无目标说明机器人贴东墙
     * 起步（左光面向空旷场地），切换镜像地图并由
     * Path_TrySetMirroredInitialPosition 锚定；左光有目标则是常规
     * 侧，继续等待左光锚定。只判定一次，判定后不再改变。
     */
    if (!path_diagnostics.initial_position_valid &&
        !path_side_detection_done &&
        front_online &&
        (Path_ClampLaserCm(front_cm) < PATH_LASER_MAX_CM))
    {
        bool left_has_target =
            (path_left_initial_sampler.count >= PATH_INITIAL_SAMPLE_COUNT) &&
            (Path_InitialMedian(&path_left_initial_sampler) <
             PATH_LASER_MAX_CM);

        path_side_detection_done = true;
        if (!left_has_target)
        {
            path_mirrored_detected = true;
        }
    }

    path_diagnostics.front_initial_sample_count = 0U;
    path_diagnostics.left_initial_sample_count =
        path_left_initial_sampler.count;
    path_diagnostics.front_distance_cm =
        Path_UpdateLaserFilter(&path_front_filter, front_last_rx,
                               front_cm, front_online);
    path_diagnostics.left_distance_cm =
        Path_UpdateLaserFilter(&path_left_filter, left_last_rx,
                               left_cm, left_online);

    if (front_online)
    {
        float front_distance_m =
            (float)path_diagnostics.front_distance_cm * 0.01f;
        if (front_distance_m <= PATH_FRONT_BASE_DISTANCE_M)
        {
            path_front_blocked = true;
        }
        else if (front_distance_m >=
                 (PATH_FRONT_BASE_DISTANCE_M +
                  PATH_BLOCK_RELEASE_HYSTERESIS_M))
        {
            path_front_blocked = false;
        }
    }
    else
    {
        /* 用户明确要求激光离线不停机；仅在诊断中保留离线状态。 */
        path_front_blocked = false;
    }

    if (left_online)
    {
        float left_distance_m =
            (float)path_diagnostics.left_distance_cm * 0.01f;
        if (left_distance_m <= PATH_LEFT_BASE_DISTANCE_M)
        {
            path_left_blocked = true;
        }
        else if (left_distance_m >=
                 (PATH_LEFT_BASE_DISTANCE_M +
                  PATH_BLOCK_RELEASE_HYSTERESIS_M))
        {
            path_left_blocked = false;
        }
    }
    else
    {
        path_left_blocked = false;
    }

    path_diagnostics.front_hard_blocked = path_front_blocked;
    path_diagnostics.left_hard_blocked = path_left_blocked;
}

static bool Path_ApplyMapLimit(int16_t *vx, int16_t *vy)
{
    float magnitude;
    float allowed_speed_mps;
    bool hard_stop = false;

    path_diagnostics.map_speed_limited = false;
    path_diagnostics.map_clearance_m = PATH_MAP_CLEARANCE_NONE_M;
    if (!path_diagnostics.odometry_valid ||
        !path_diagnostics.initial_position_valid ||
        ((*vx == 0) && (*vy == 0)))
    {
        return false;
    }

    magnitude = sqrtf((float)*vx * (float)*vx +
                      (float)*vy * (float)*vy);
    path_diagnostics.map_clearance_m =
        PathMap_RayClearance(path_diagnostics.map_x_m,
                             path_diagnostics.map_y_m,
                             (float)*vx / magnitude,
                             (float)*vy / magnitude);
    if (path_diagnostics.map_clearance_m >=
        PATH_MAP_CLEARANCE_NONE_M)
    {
        return false;
    }

    allowed_speed_mps =
        PathSafety_MaxAllowedSpeed(path_diagnostics.map_clearance_m,
                                   0.0f, PATH_REACTION_TIME_S,
                                   PATH_MAP_BRAKE_DECELERATION_MPS2);
    if (PathSafety_LimitVectorCommand(vx, vy, allowed_speed_mps,
                                      PATH_COMMAND_TO_MPS) != 0)
    {
        path_diagnostics.map_speed_limited = true;
    }
    if (path_diagnostics.map_clearance_m <=
        PATH_MAP_HARD_STOP_DISTANCE_M)
    {
        hard_stop = true;
        *vx = 0;
        *vy = 0;
    }
    return hard_stop;
}

static bool Path_ApplyLaserLimits(int16_t *vx, int16_t *vy)
{
    float requested_speed_mps;
    float measured_speed_mps;
    float safety_speed_mps;
    float distance_m;
    int16_t limited;
    bool hard_stop = false;

    path_diagnostics.front_speed_limited = false;
    path_diagnostics.left_speed_limited = false;
    path_diagnostics.front_required_distance_m = PATH_FRONT_BASE_DISTANCE_M;
    path_diagnostics.left_required_distance_m = PATH_LEFT_BASE_DISTANCE_M;
    path_diagnostics.front_allowed_speed_mps = 0.0f;
    path_diagnostics.left_allowed_speed_mps = 0.0f;

    if (path_diagnostics.front_laser_online)
    {
        distance_m = (float)path_diagnostics.front_distance_cm * 0.01f;
        requested_speed_mps = (*vy > 0) ?
            (float)*vy * PATH_COMMAND_TO_MPS : 0.0f;
        measured_speed_mps = Path_MaxFloat(
            path_diagnostics.encoder_velocity_y_mps, 0.0f);
        safety_speed_mps = Path_MaxFloat(requested_speed_mps,
                                         measured_speed_mps);
        path_diagnostics.front_required_distance_m =
            PathSafety_RequiredDistance(safety_speed_mps,
                                        PATH_FRONT_BASE_DISTANCE_M,
                                        PATH_REACTION_TIME_S,
                                        PATH_BRAKE_DECELERATION_MPS2);
        path_diagnostics.front_allowed_speed_mps =
            PathSafety_MaxAllowedSpeed(distance_m,
                                       PATH_FRONT_BASE_DISTANCE_M,
                                       PATH_REACTION_TIME_S,
                                       PATH_BRAKE_DECELERATION_MPS2);
        if (*vy > 0)
        {
            limited = PathSafety_LimitAxisCommand(
                *vy, path_diagnostics.front_allowed_speed_mps,
                PATH_COMMAND_TO_MPS);
            if (limited != *vy)
            {
                path_diagnostics.front_speed_limited = true;
                *vy = limited;
            }
        }
        if (path_front_blocked && (*vy > 0))
        {
            path_diagnostics.front_speed_limited = true;
            *vy = 0;
        }
        /*
         * 实测仍在逼近且已高于当前距离允许值时立即制动；若进入硬阻挡
         * 时上一安全输出仍向前，即使里程计暂时无效也制动。负向 vy 是
         * 主动远离前障碍，保留恢复；纯横移不能掩盖前向惯性。
         */
        if ((*vy >= 0) &&
            ((measured_speed_mps >
              (path_diagnostics.front_allowed_speed_mps +
               PATH_HARD_STOP_SPEED_MPS)) ||
             (path_front_blocked && (path_last_output_vy > 0))))
        {
            hard_stop = true;
        }
    }

    /*
     * 常规侧左光面向西墙，20 cm 饱和仍按“墙在量程边界”保守限速；
     * 镜像侧左光面向空旷场地（西墙在 2.4 m 外），只有出现真实目标
     * （< 20 cm，例如对抗中靠近的另一台车）时才启用左光限速。
     */
    if (path_diagnostics.left_laser_online &&
        (!PathMap_IsMirrored() ||
         (path_diagnostics.left_distance_cm < PATH_LASER_MAX_CM)))
    {
        distance_m = (float)path_diagnostics.left_distance_cm * 0.01f;
        requested_speed_mps = (*vx < 0) ?
            -(float)*vx * PATH_COMMAND_TO_MPS : 0.0f;
        measured_speed_mps = Path_MaxFloat(
            -path_diagnostics.encoder_velocity_x_mps, 0.0f);
        safety_speed_mps = Path_MaxFloat(requested_speed_mps,
                                         measured_speed_mps);
        path_diagnostics.left_required_distance_m =
            PathSafety_RequiredDistance(safety_speed_mps,
                                        PATH_LEFT_BASE_DISTANCE_M,
                                        PATH_REACTION_TIME_S,
                                        PATH_BRAKE_DECELERATION_MPS2);
        path_diagnostics.left_allowed_speed_mps =
            PathSafety_MaxAllowedSpeed(distance_m,
                                       PATH_LEFT_BASE_DISTANCE_M,
                                       PATH_REACTION_TIME_S,
                                       PATH_BRAKE_DECELERATION_MPS2);
        if (*vx < 0)
        {
            limited = PathSafety_LimitAxisCommand(
                *vx, path_diagnostics.left_allowed_speed_mps,
                PATH_COMMAND_TO_MPS);
            if (limited != *vx)
            {
                path_diagnostics.left_speed_limited = true;
                *vx = limited;
            }
        }
        if (path_left_blocked && (*vx < 0))
        {
            path_diagnostics.left_speed_limited = true;
            *vx = 0;
        }
        /* 负向实测速度逼近左障碍；+X 逃离命令保持可用。 */
        if ((*vx <= 0) &&
            ((measured_speed_mps >
              (path_diagnostics.left_allowed_speed_mps +
               PATH_HARD_STOP_SPEED_MPS)) ||
             (path_left_blocked && (path_last_output_vx < 0))))
        {
            hard_stop = true;
        }
    }

    return hard_stop;
}

static void Path_ApplyOutput(int16_t vx, int16_t vy, bool force_stop)
{
    if (force_stop)
    {
        Chassis_StopAll();
        vx = 0;
        vy = 0;
    }
    else if ((vx != path_last_output_vx) ||
             (vy != path_last_output_vy) ||
             (path_last_output_z != 0))
    {
        (void)Chassis_SetVelocity(vx, vy, 0);
    }

    path_last_output_vx = vx;
    path_last_output_vy = vy;
    path_last_output_z = 0;
    path_diagnostics.output_vx = vx;
    path_diagnostics.output_vy = vy;
    path_diagnostics.output_z = 0;
}

void Path_Init(void)
{
    (void)memset(&path_diagnostics, 0, sizeof(path_diagnostics));
    (void)memset(&path_front_filter, 0, sizeof(path_front_filter));
    (void)memset(&path_left_filter, 0, sizeof(path_left_filter));
    (void)memset(&path_front_initial_sampler, 0,
                 sizeof(path_front_initial_sampler));
    (void)memset(&path_left_initial_sampler, 0,
                 sizeof(path_left_initial_sampler));
    path_remote_vx = 0;
    path_remote_vy = 0;
    path_remote_buttons = 0U;
    path_remote_timestamp_ms = 0U;
    path_remote_online = 0U;
    path_remote_sequence = 0U;
    path_last_output_vx = 0;
    path_last_output_vy = 0;
    path_last_output_z = 0;
    path_processed_remote_sequence = 0U;
    path_map_origin_x_m = 0.0f;
    path_map_origin_y_m = 0.0f;
    path_localization_yaw_deg = 0.0f;
    path_localization_yaw_valid = false;
    path_mode_button_armed = true;
    path_yaw_was_ready = false;
    path_front_blocked = false;
    path_left_blocked = false;
    path_side_detection_done = false;
    path_mirrored_detected = false;
    PathMap_SetMirrored(false);

    path_diagnostics.initialized = true;
    path_diagnostics.segment_count = PATH_MAP_ROUTE_SEGMENT_COUNT;
    path_diagnostics.active_axis = PATH_MAP_AXIS_Y;
    path_diagnostics.map_clearance_m = PATH_MAP_CLEARANCE_NONE_M;
}

void Path_SubmitRemoteCommand(int16_t *vx, int16_t *vy, int16_t *z,
                              uint8_t six_buttons, uint32_t now_ms)
{
    int16_t raw_vx;
    int16_t raw_vy;

    if ((vx == NULL) || (vy == NULL) || (z == NULL))
    {
        return;
    }

    raw_vx = *vx;
    raw_vy = *vy;
    Path_WriteRemoteMailbox(raw_vx, raw_vy,
                            (uint8_t)(six_buttons & 0x3FU),
                            now_ms, true);

    /* 原 LoRa 调用保留，但只能重复最近一次已通过 1 ms 安全层的输出。 */
    *vx = path_last_output_vx;
    *vy = path_last_output_vy;
    *z = 0;
}

void Path_NotifyRemoteOffline(uint32_t now_ms)
{
    Path_WriteRemoteMailbox(0, 0, 0U, now_ms, false);
    path_last_output_vx = 0;
    path_last_output_vy = 0;
    path_last_output_z = 0;
}

void Path_ReplaceNonRemoteCommand(int16_t *vx, int16_t *vy, int16_t *z)
{
    if ((vx == NULL) || (vy == NULL) || (z == NULL))
    {
        return;
    }
    *vx = path_last_output_vx;
    *vy = path_last_output_vy;
    *z = 0;
}

void Path_Run1ms(uint32_t now_ms)
{
    path_remote_snapshot_t remote;
    int16_t vx;
    int16_t vy;
    bool automatic_stop;
    bool hard_stop;

    if (!path_diagnostics.initialized)
    {
        return;
    }

    Path_ReadRemoteMailbox(&remote);
    if (remote.online &&
        ((uint32_t)(now_ms - remote.timestamp_ms) > PATH_REMOTE_TIMEOUT_MS))
    {
        remote.online = false;
    }
    Path_ProcessModeButton(&remote);
    Path_UpdateYawZeroLock();
    Path_UpdateLaserData();
    automatic_stop = Path_UpdateOdometryAndRoute();
    path_diagnostics.map_mirrored = PathMap_IsMirrored();

    path_diagnostics.remote_online = remote.online;
    path_diagnostics.last_remote_ms = remote.timestamp_ms;
    path_diagnostics.raw_vx = remote.vx;
    path_diagnostics.raw_vy = remote.vy;

    if (!remote.online)
    {
        path_diagnostics.single_axis_enabled = false;
        path_mode_button_armed = false;
        remote.vx = 0;
        remote.vy = 0;
    }

    if (path_diagnostics.neutral_rearm_required)
    {
        if ((Path_AbsCommand(remote.vx) <= PATH_NEUTRAL_COMMAND_THRESHOLD) &&
            (Path_AbsCommand(remote.vy) <= PATH_NEUTRAL_COMMAND_THRESHOLD))
        {
            path_diagnostics.neutral_rearm_required = false;
        }
        else
        {
            remote.vx = 0;
            remote.vy = 0;
        }
    }

    vx = remote.vx;
    vy = remote.vy;
    if (path_diagnostics.single_axis_enabled)
    {
        if (path_diagnostics.active_axis == PATH_MAP_AXIS_X)
        {
            vy = 0;
        }
        else
        {
            vx = 0;
        }
    }

    hard_stop = Path_ApplyMapLimit(&vx, &vy);
    hard_stop = Path_ApplyLaserLimits(&vx, &vy) || hard_stop;

    /*
     * 高优先级通信任务若在本轮计算中更新了邮箱，不得回写旧命令。
     * 比较、底盘提交和 last_output 更新必须不可抢占，否则通信任务可能
     * 在比较后插入新帧，随后又被本轮旧快照覆盖，或读到一半的新输出。
     */
    {
        uint32_t primask = __get_PRIMASK();

        __disable_irq();
        __DMB();
        if (path_remote_sequence != remote.sequence)
        {
            if (automatic_stop || hard_stop)
            {
                Path_ApplyOutput(0, 0, true);
            }
            if (primask == 0U)
            {
                __enable_irq();
            }
            return;
        }
        Path_ApplyOutput(vx, vy, automatic_stop || hard_stop);
        if (primask == 0U)
        {
            __enable_irq();
        }
    }
}

bool Path_GetDiagnostics(path_diagnostics_t *diagnostics)
{
    if (!path_diagnostics.initialized || (diagnostics == NULL))
    {
        return false;
    }
    *diagnostics = path_diagnostics;
    return true;
}
