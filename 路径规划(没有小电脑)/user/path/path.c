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
/*
 * 回程"贴面回到起点"的前光判定阈值，纯几何推导：
 * - 前 DT35 凹进车头面 半车长−安装偏移 = 0.3085−0.225 = 8.35 cm，
 *   正对贴面时读数下限 8 cm（旧阈值 5 cm 要求车头钻进墙里
 *   2.4 cm，物理上永远无法触发）；
 * - 允许 PATH_MAP_RETURN_ALIGN_DEG = 5° 的掉头残差：斜置时贴面
 *   读数为 recess·cos5° + 半车宽·sin5° = 10.24 cm；
 * 取整后阈值 10 cm：正对时在贴面前 ≤2.7 cm 处判定回到起点，
 * 5° 斜置时恰在贴面接触时判定，随后强制归零吸收该残差。
 */
#define PATH_RETURN_ALIGN_COS 0.99619f
#define PATH_RETURN_ALIGN_SIN 0.08716f
#define PATH_RETURN_HOME_FRONT_CM                                   \
    ((uint16_t)(((PATH_MAP_INITIAL_CENTER_Y_M -                     \
                  PATH_LOCALIZATION_FRONT_SENSOR_OFFSET_M) *        \
                 PATH_RETURN_ALIGN_COS +                            \
                 0.5f * PATH_MAP_ROBOT_WIDTH_M *                    \
                 PATH_RETURN_ALIGN_SIN) * 100.0f))
/*
 * 回到起点的位置容差：倒车回程（车尾朝墙）没有激光可校正/兜底，
 * 末段完全依赖里程计，而镜像侧锚定本身带有 DT35 整 cm 截断引入的
 * 最多 1 cm 系统偏置——零容差会让物理已贴靠的机器人永远差 1~2 mm
 * 无法判定回到起点。1 cm 与锚定量化误差同级。
 */
#define PATH_RETURN_HOME_POS_TOL_M      0.010f
/*
 * 回程末段贴靠时的前光基础净距：常规 12 cm 基距大于贴靠读数下限
 * 8.35 cm，会在离起点 3~4 cm 处触发前光硬阻挡，永远无法贴靠回
 * 起点。末段改用贴靠读数下限本身作基距：允许速度随读数逼近
 * 8 cm 平滑收敛到 0，读数到 8 cm 时恰好触发回到起点判定。
 */
#define PATH_RETURN_DOCK_FRONT_BASE_M                               \
    (PATH_MAP_INITIAL_CENTER_Y_M - PATH_LOCALIZATION_FRONT_SENSOR_OFFSET_M)
/* 距段终点小于该距离时自动解除单轴约束（不停车、不要求回中）。 */
#define PATH_AXIS_AUTO_RELEASE_M        0.100f

typedef struct
{
    int16_t vx;
    int16_t vy;
    int16_t vz;
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
static volatile int16_t path_remote_vz;
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
static bool path_return_button_armed;
/*
 * 回程锁定航向：进入回程时按车头朝向自动选择——车头靠近 +Y
 * （|yaw|<=90°）则倒车回程（锁 0°），靠近 -Y 则车头朝 -Y 前进
 * （锁 180°）。运行中若实测 yaw 偏离当前锁定角超过 100°（含
 * 10° 滞回，例如驾驶员用肩键调头），自动切换到另一朝向。
 */
static float path_return_lock_yaw_deg;
static bool path_return_target_pending;
/*
 * 起点位姿（回程归零目标）。常规侧 = 锚定结果（锚定就发生在起点）；
 * 镜像侧锚定发生在行进途中第一堵前墙处，起点必须另取
 * (PATH_MAP_MIRRORED_START_X_M, PATH_MAP_INITIAL_CENTER_Y_M)，
 * 否则回程归零会把地图坐标错误平移到锚定点（约 +1.35 m）。
 */
static float path_home_map_x_m;
static float path_home_map_y_m;

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

static float Path_AngleDiffDeg(float lhs_deg, float rhs_deg)
{
    float diff = fmodf(lhs_deg - rhs_deg, 360.0f);

    if (diff > 180.0f)
    {
        diff -= 360.0f;
    }
    else if (diff < -180.0f)
    {
        diff += 360.0f;
    }
    return diff;
}

/*
 * 车体系与地图系互转。Chassis_SetVelocity 的 vx/vy 是车体系
 * （chassis_main.c："机器人坐标系：X 向右、Y 向前"），掉头 180°
 * 后车体系相对地图系整体取反；所有涉及地图几何的计算都必须先用
 * 实测 yaw 把命令旋到地图系，反之亦然。yaw 无效时回退到锁定目标
 * 角（去程 0°/回程 180°）。
 */
static float Path_CommandYawDeg(void)
{
    if (path_localization_yaw_valid)
    {
        return path_localization_yaw_deg;
    }
    return path_diagnostics.return_mode ? path_return_lock_yaw_deg
                                        : PATH_MAP_LOCK_YAW_DEG;
}

static void Path_RotateVector(float x, float y, float yaw_deg,
                              float *out_x, float *out_y)
{
    float yaw_rad = yaw_deg * (PATH_PI / 180.0f);
    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);

    *out_x = cos_yaw * x - sin_yaw * y;
    *out_y = sin_yaw * x + cos_yaw * y;
}

/* 回程末段贴靠起点时前光使用贴靠基距，其余时间保持 12 cm。 */
static float Path_FrontBaseDistanceM(void)
{
    if (path_diagnostics.return_mode &&
        !path_diagnostics.route_complete &&
        (path_diagnostics.segment_index ==
         (PATH_MAP_ROUTE_SEGMENT_COUNT - 1U)))
    {
        return PATH_RETURN_DOCK_FRONT_BASE_M;
    }
    return PATH_FRONT_BASE_DISTANCE_M;
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

static void Path_WriteRemoteMailbox(int16_t vx, int16_t vy, int16_t vz,
                                    uint8_t buttons, uint32_t now_ms,
                                    bool online)
{
    uint32_t sequence = path_remote_sequence;

    path_remote_sequence = sequence + 1U;
    __DMB();
    path_remote_vx = vx;
    path_remote_vy = vy;
    path_remote_vz = vz;
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
        snapshot->vz = path_remote_vz;
        snapshot->buttons = path_remote_buttons;
        snapshot->timestamp_ms = path_remote_timestamp_ms;
        snapshot->online = path_remote_online != 0U;
        __DMB();
        sequence_after = path_remote_sequence;
    } while (((sequence_before & 1U) != 0U) ||
             (sequence_before != sequence_after));
    snapshot->sequence = sequence_after;
}

static bool Path_ProcessModeButton(const path_remote_snapshot_t *remote)
{
    const path_map_route_segment_t *route;
    bool pressed;
    bool return_pressed;
    bool require_stop = false;
    uint8_t route_count;

    if (remote->sequence == path_processed_remote_sequence)
    {
        return false;
    }
    path_processed_remote_sequence = remote->sequence;
    pressed = (remote->buttons & PATH_REMOTE_MODE_BUTTON_BIT) != 0U;
    return_pressed =
        (remote->buttons & PATH_REMOTE_RETURN_BUTTON_BIT) != 0U;

    if (!remote->online)
    {
        path_diagnostics.single_axis_enabled = false;
        path_mode_button_armed = false;
        path_return_button_armed = false;
        return false;
    }
    if (!pressed)
    {
        path_mode_button_armed = true;
    }
    if (!return_pressed)
    {
        path_return_button_armed = true;
    }

    if (pressed && path_mode_button_armed)
    {
        path_mode_button_armed = false;
        if (path_diagnostics.segment_index <
            PATH_MAP_ROUTE_SEGMENT_COUNT)
        {
            path_diagnostics.single_axis_enabled =
                !path_diagnostics.single_axis_enabled;
            path_diagnostics.manual_toggle_count++;
            route = PathMap_GetReturnRoute(&route_count);
            if (path_diagnostics.return_mode)
            {
                path_diagnostics.active_axis =
                    route[path_diagnostics.segment_index].axis;
            }
            else
            {
                route = PathMap_GetRoute(&route_count);
                if (path_diagnostics.segment_index < route_count)
                {
                    path_diagnostics.active_axis =
                        route[path_diagnostics.segment_index].axis;
                }
            }
        }
    }

    /*
     * 回程键（按键 2）：去程走完后按下进入回程（yaw 目标 180°、
     * 路线反转、段归 0）；回程走完（return_complete）后再按退出
     * 回程、重新开始去程（yaw 目标 0°）。两种切换都要求摇杆回中。
     * 去程走完后允许继续前进完成任务，之后再按本键回程。
     */
    if (return_pressed && path_return_button_armed)
    {
        path_return_button_armed = false;
        if (!path_diagnostics.return_mode &&
            path_diagnostics.route_complete &&
            path_diagnostics.initial_position_valid)
        {
            path_diagnostics.return_mode = true;
            path_diagnostics.return_complete = false;
            path_diagnostics.single_axis_enabled = false;
            path_diagnostics.segment_index = 0U;
            path_diagnostics.route_complete = false;
            path_diagnostics.neutral_rearm_required = true;
            /*
             * 自动检测车头朝向：靠近 +Y（|yaw|<=90°）→ 倒车回程
             * （锁 0°，无需掉头）；靠近 -Y → 车头朝 -Y 前进
             * （锁 180°）。yaw 无效时按倒车处理（不需要旋转）。
             */
            if (path_localization_yaw_valid &&
                (fabsf(Path_AngleDiffDeg(path_localization_yaw_deg,
                                         PATH_MAP_LOCK_YAW_DEG)) > 90.0f))
            {
                path_return_lock_yaw_deg = PATH_MAP_RETURN_YAW_DEG;
            }
            else
            {
                path_return_lock_yaw_deg = PATH_MAP_LOCK_YAW_DEG;
            }
            path_return_target_pending = true;
            PathMap_SetReturnMode(true);
            require_stop = true;
        }
        else if (path_diagnostics.return_mode &&
                 path_diagnostics.return_complete)
        {
            path_diagnostics.return_mode = false;
            path_diagnostics.return_complete = false;
            path_diagnostics.single_axis_enabled = false;
            path_diagnostics.segment_index = 0U;
            path_diagnostics.route_complete = false;
            path_diagnostics.neutral_rearm_required = true;
            PathMap_SetReturnMode(false);
            require_stop = true;
        }
    }

    return require_stop;
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
    path_home_map_x_m = PATH_MAP_MIRRORED_START_X_M;
    path_home_map_y_m = PATH_MAP_INITIAL_CENTER_Y_M;
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
    path_home_map_x_m = result.map_x_m;
    path_home_map_y_m = result.map_y_m;
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

    /*
     * 回程最后一段（Y- 到起点）：车头朝 -Y，前 DT35 朝起始贴靠面，
     * 接近起点时用前光距离实时校正 Y（吸收里程计 Y 漂移）。校正
     * 基准是初始摆位的贴靠面 PATH_MAP_START_FACE_Y_M（=0，初始
     * 中心 Y=车长一半即由该面得出）；不能再叠加 0.049 m 墙厚，
     * 否则校正值系统性偏大一个墙厚，段终点 0.3085 永远无法到达。
     * 镜像侧左 DT35 朝东墙，读到东墙时实时校正 X。
     * 校正发生在 map 计算之前，本帧立即生效。
     */
    if (path_diagnostics.return_mode &&
        (path_diagnostics.segment_index ==
         (PATH_MAP_ROUTE_SEGMENT_COUNT - 1U)))
    {
        /* 倒车回程（车头朝 +Y）时前光朝外、镜像左光朝西侧空旷，
         * 激光校正不可用，末段只依赖里程计。 */
        bool facing_south =
            cosf(Path_CommandYawDeg() * (PATH_PI / 180.0f)) < 0.0f;

        if (facing_south &&
            path_diagnostics.front_laser_online &&
            (path_diagnostics.front_distance_cm < PATH_LASER_MAX_CM))
        {
            float front_distance_m =
                (float)path_diagnostics.front_distance_cm * 0.01f;
            float y_corrected_m = PATH_MAP_START_FACE_Y_M +
                                  PATH_LOCALIZATION_FRONT_SENSOR_OFFSET_M +
                                  front_distance_m;

            path_map_origin_y_m = y_corrected_m -
                                  odometry.fused_position_y_m;
        }
        if (facing_south &&
            PathMap_IsMirrored() &&
            path_diagnostics.left_laser_online &&
            (path_diagnostics.left_distance_cm < PATH_LASER_MAX_CM))
        {
            float left_distance_m =
                (float)path_diagnostics.left_distance_cm * 0.01f;
            float x_corrected_m = PATH_MAP_EAST_INNER_X_M -
                                  PATH_LOCALIZATION_LEFT_SENSOR_OFFSET_M -
                                  left_distance_m;

            path_map_origin_x_m = x_corrected_m -
                                  odometry.fused_position_x_m;
        }
    }

    path_diagnostics.map_x_m = path_map_origin_x_m +
                               odometry.fused_position_x_m;
    path_diagnostics.map_y_m = path_map_origin_y_m +
                               odometry.fused_position_y_m;

    route = path_diagnostics.return_mode ? PathMap_GetReturnRoute(&route_count)
                                         : PathMap_GetRoute(&route_count);
    /*
     * 单轴自动解除：距当前段终点不足 PATH_AXIS_AUTO_RELEASE_M 时
     * 提前解除约束（不强停、不要求摇杆回中），驾驶员可以直接把
     * 摇杆过渡到下一段方向；越过终点时同样只解除不停车。
     */
    if (path_diagnostics.single_axis_enabled &&
        (path_diagnostics.segment_index < route_count))
    {
        const path_map_route_segment_t *segment =
            &route[path_diagnostics.segment_index];
        float coordinate = (segment->axis == PATH_MAP_AXIS_X) ?
                           path_diagnostics.map_x_m :
                           path_diagnostics.map_y_m;
        float remaining_m = (segment->direction > 0) ?
                            (segment->target_m - coordinate) :
                            (coordinate - segment->target_m);

        if (remaining_m <= PATH_AXIS_AUTO_RELEASE_M)
        {
            path_diagnostics.single_axis_enabled = false;
            path_diagnostics.automatic_cancel_count++;
        }
    }
    while ((path_diagnostics.segment_index < route_count) &&
           PathMap_SegmentReached(path_diagnostics.segment_index,
                                  path_diagnostics.map_x_m,
                                  path_diagnostics.map_y_m))
    {
        path_diagnostics.segment_index++;
        if (path_diagnostics.single_axis_enabled)
        {
            path_diagnostics.single_axis_enabled = false;
            path_diagnostics.automatic_cancel_count++;
        }
    }

    path_diagnostics.route_complete =
        path_diagnostics.segment_index >= route_count;

    /*
     * 回程末段贴靠起点：南向边界安全线临时收窄（否则 0.3285 m
     * 净空线把机器人挡在起点前 3 cm 处）；离开末段或完成后恢复。
     */
    PathMap_SetFinalApproach(path_diagnostics.return_mode &&
                             !path_diagnostics.route_complete &&
                             (path_diagnostics.segment_index ==
                              (PATH_MAP_ROUTE_SEGMENT_COUNT - 1U)));

    /*
     * 回程走完 = 回到起点：把地图坐标强制归零到初始锚定位置，
     * 吸收全程里程计漂移，之后可以再按回程键重新开始去程。
     * 除段终点外，还提供"贴南墙"物理兜底：最后一段前光读数降到
     * PATH_RETURN_HOME_FRONT_CM（8 cm，= 半车长 − 前光安装偏移，
     * 即车头面贴到起始贴靠面时的读数下限）且地图 Y 在起点附近时，
     * 直接判定回到起点，避免整 cm 截断导致段终点差 1~2 cm 无法
     * 触发。旧阈值 5 cm 需要车头面进入墙体 2.4 cm，物理不可达。
     */
    if (path_diagnostics.return_mode &&
        !path_diagnostics.return_complete &&
        (path_diagnostics.route_complete ||
         ((path_diagnostics.segment_index ==
           (PATH_MAP_ROUTE_SEGMENT_COUNT - 1U)) &&
          (((cosf(Path_CommandYawDeg() * (PATH_PI / 180.0f)) < 0.0f) &&
            path_diagnostics.front_laser_online &&
            (path_diagnostics.front_distance_cm <=
             PATH_RETURN_HOME_FRONT_CM) &&
            (path_diagnostics.map_y_m <=
             (PATH_MAP_INITIAL_CENTER_Y_M + 0.120f))) ||
           (path_diagnostics.map_y_m <=
            (PATH_MAP_INITIAL_CENTER_Y_M +
             PATH_RETURN_HOME_POS_TOL_M))))))
    {
        /* 归零到起点位姿（镜像侧不能用中途的锚定点）。 */
        path_map_origin_x_m = path_home_map_x_m -
                              odometry.fused_position_x_m;
        path_map_origin_y_m = path_home_map_y_m -
                              odometry.fused_position_y_m;
        path_diagnostics.map_x_m = path_home_map_x_m;
        path_diagnostics.map_y_m = path_home_map_y_m;
        path_diagnostics.return_complete = true;
        path_diagnostics.single_axis_enabled = false;
        path_diagnostics.neutral_rearm_required = true;
        automatic_stop = true;
    }

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
    float target_yaw_deg;
    float yaw_error_deg;
    bool assert_target;
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

    /*
     * 回程中持续检测车头朝向：实测 yaw 偏离当前锁定角超过 100°
     * （例如驾驶员用肩键调头）时切换到另一朝向，倒车/正向随之
     * 互换；100° 相对 90° 分界留 10° 滞回避免抖动。
     */
    if (ready && path_diagnostics.return_mode &&
        (fabsf(Path_AngleDiffDeg(imu.yaw_deg,
                                 path_return_lock_yaw_deg)) > 100.0f))
    {
        path_return_lock_yaw_deg =
            (path_return_lock_yaw_deg == PATH_MAP_LOCK_YAW_DEG) ?
            PATH_MAP_RETURN_YAW_DEG : PATH_MAP_LOCK_YAW_DEG;
        path_return_target_pending = true;
    }

    target_yaw_deg = path_diagnostics.return_mode ?
                     path_return_lock_yaw_deg : PATH_MAP_LOCK_YAW_DEG;
    /*
     * 目标角写入策略：去程持续锁 0°（肩键被本层清零，不会被
     * 改写）；回程只在进入/切向/恢复时写入一次，之后允许驾驶员
     * 用肩键微调航向（CalcOmega 手动旋转会把保持目标改到当前
     * 角），本层不再反复回拉。
     */
    assert_target = !path_yaw_was_ready || !imu.yaw_hold_enabled;
    if (path_diagnostics.return_mode)
    {
        assert_target = assert_target || path_return_target_pending;
    }
    else
    {
        assert_target = assert_target ||
                        (fabsf(Path_AngleDiffDeg(imu.target_yaw_deg,
                                                 target_yaw_deg)) > 0.5f);
    }
    if (ready && assert_target)
    {
        ImuMain_EnableYawHold(true);
        if (ImuMain_SetTargetYaw(target_yaw_deg) != HAL_OK)
        {
            ready = false;
        }
        else
        {
            path_return_target_pending = false;
        }
    }
    path_yaw_was_ready = ready;

    yaw_error_deg = ready ?
                    Path_AngleDiffDeg(imu.yaw_deg, target_yaw_deg) : 0.0f;
    path_diagnostics.return_yaw_aligned =
        ready && (fabsf(yaw_error_deg) <= PATH_MAP_RETURN_ALIGN_DEG);
    path_diagnostics.yaw_zero_lock_ready = ready;
    path_diagnostics.return_reverse =
        path_diagnostics.return_mode &&
        (path_return_lock_yaw_deg == PATH_MAP_LOCK_YAW_DEG);
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
        float front_base_m = Path_FrontBaseDistanceM();

        if (front_distance_m <= front_base_m)
        {
            path_front_blocked = true;
        }
        else if (front_distance_m >=
                 (front_base_m + PATH_BLOCK_RELEASE_HYSTERESIS_M))
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
    float map_dir_x;
    float map_dir_y;
    float command_yaw_deg;
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
    /*
     * vx/vy 是车体系命令，地图净空必须沿地图系方向查询：掉头 180°
     * 后车体系相对地图系取反，直接用车体方向会把净空查到运动的
     * 反方向——回程时对真实运动方向完全失去静态墙保护，反而在
     * 安全方向上误触发限速/硬停。
     */
    command_yaw_deg = Path_CommandYawDeg();
    Path_RotateVector((float)*vx / magnitude, (float)*vy / magnitude,
                      command_yaw_deg, &map_dir_x, &map_dir_y);
    path_diagnostics.map_clearance_m =
        PathMap_RayClearance(path_diagnostics.map_x_m,
                             path_diagnostics.map_y_m,
                             map_dir_x, map_dir_y);

    /*
     * 沿墙滑动：主方向被挡（例如中心已在膨胀带内，命令带有极小的
     * 向墙分量）时，把命令旋到地图系只保留主轴分量再查一次净空；
     * 可行则以投影后的命令继续（清除向墙分量），不可行才硬停。
     * 否则一次越界就会把机器人锁死在膨胀带内。
     */
    if (path_diagnostics.map_clearance_m <= PATH_MAP_HARD_STOP_DISTANCE_M)
    {
        float map_cmd_x;
        float map_cmd_y;
        float body_cmd_x;
        float body_cmd_y;
        float slide_clearance_m;

        Path_RotateVector((float)*vx, (float)*vy, command_yaw_deg,
                          &map_cmd_x, &map_cmd_y);
        if (fabsf(map_cmd_x) >= fabsf(map_cmd_y))
        {
            map_cmd_y = 0.0f;
        }
        else
        {
            map_cmd_x = 0.0f;
        }
        slide_clearance_m = PathMap_RayClearance(
            path_diagnostics.map_x_m, path_diagnostics.map_y_m,
            map_cmd_x, map_cmd_y);
        if (((map_cmd_x != 0.0f) || (map_cmd_y != 0.0f)) &&
            (slide_clearance_m > PATH_MAP_HARD_STOP_DISTANCE_M))
        {
            Path_RotateVector(map_cmd_x, map_cmd_y, -command_yaw_deg,
                              &body_cmd_x, &body_cmd_y);
            *vx = (int16_t)((body_cmd_x >= 0.0f) ? (body_cmd_x + 0.5f)
                                                 : (body_cmd_x - 0.5f));
            *vy = (int16_t)((body_cmd_y >= 0.0f) ? (body_cmd_y + 0.5f)
                                                 : (body_cmd_y - 0.5f));
            path_diagnostics.map_speed_limited = true;
            path_diagnostics.map_clearance_m = slide_clearance_m;
            magnitude = sqrtf((float)*vx * (float)*vx +
                              (float)*vy * (float)*vy);
            if (magnitude <= 0.0f)
            {
                return false;
            }
        }
    }
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
    /*
     * vx/vy 与 DT35 都在车体系：前光装在车头（车体 +Y），左光装在
     * 左侧（车体 -X），无论车头朝哪，前光永远只限制 vy>0、左光
     * 永远只限制 vx<0。回程"保护地图 -Y / +X"由车体系自然成立
     * （车头朝 -Y 后车体 +Y 即地图 -Y），不需要也不允许对命令取
     * 反——原实现在回程把命令按地图系取反，真实逼近墙时反而完全
     * 不限速。returning/镜像标志只用于判断左光当前物理上面向墙
     * 还是空旷场地（20 cm 饱和是否当真）。
     */
    int32_t front_command;
    int32_t left_command;
    int32_t limited;
    float requested_speed_mps;
    float measured_speed_mps;
    float safety_speed_mps;
    float distance_m;
    bool hard_stop = false;
    bool left_gate;

    float front_base_m = Path_FrontBaseDistanceM();

    path_diagnostics.front_speed_limited = false;
    path_diagnostics.left_speed_limited = false;
    path_diagnostics.front_required_distance_m = front_base_m;
    path_diagnostics.left_required_distance_m = PATH_LEFT_BASE_DISTANCE_M;
    path_diagnostics.front_allowed_speed_mps = 0.0f;
    path_diagnostics.left_allowed_speed_mps = 0.0f;

    front_command = (int32_t)(*vy);
    left_command = (int32_t)-(*vx);

    if (path_diagnostics.front_laser_online)
    {
        distance_m = (float)path_diagnostics.front_distance_cm * 0.01f;
        requested_speed_mps = (front_command > 0) ?
            (float)front_command * PATH_COMMAND_TO_MPS : 0.0f;
        /* 编码器速度同样是车体系：朝前光逼近恒为 +Y 分量。 */
        measured_speed_mps = Path_MaxFloat(
            path_diagnostics.encoder_velocity_y_mps, 0.0f);
        safety_speed_mps = Path_MaxFloat(requested_speed_mps,
                                         measured_speed_mps);
        path_diagnostics.front_required_distance_m =
            PathSafety_RequiredDistance(safety_speed_mps,
                                        front_base_m,
                                        PATH_REACTION_TIME_S,
                                        PATH_BRAKE_DECELERATION_MPS2);
        path_diagnostics.front_allowed_speed_mps =
            PathSafety_MaxAllowedSpeed(distance_m,
                                       front_base_m,
                                       PATH_REACTION_TIME_S,
                                       PATH_BRAKE_DECELERATION_MPS2);
        if (front_command > 0)
        {
            limited = PathSafety_LimitAxisCommand(
                (int16_t)front_command,
                path_diagnostics.front_allowed_speed_mps,
                PATH_COMMAND_TO_MPS);
            if (limited != (int16_t)front_command)
            {
                path_diagnostics.front_speed_limited = true;
                front_command = limited;
            }
        }
        if (path_front_blocked && (front_command > 0))
        {
            path_diagnostics.front_speed_limited = true;
            front_command = 0;
        }
        /*
         * 实测仍在逼近且已高于当前距离允许值时立即制动；若进入硬阻挡
         * 时上一安全输出仍朝传感器方向，即使里程计暂时无效也制动。
         * 反方向命令是主动远离障碍，保留恢复；纯横移不能掩盖惯性。
         */
        if ((front_command >= 0) &&
            ((measured_speed_mps >
              (path_diagnostics.front_allowed_speed_mps +
               PATH_HARD_STOP_SPEED_MPS)) ||
             (path_front_blocked && (path_last_output_vy > 0))))
        {
            hard_stop = true;
        }
    }

    /*
     * 左光门控：左光面向本侧近墙（常规侧=西墙、镜像侧=东墙）时，
     * 20 cm 饱和仍按“墙在量程边界”保守限速；面向空旷场地时只有
     * 出现真实目标（< 20 cm，例如对抗中靠近的另一台车）才启用
     * 左光限速。面向由车头朝向决定：车头靠近 +Y 时左光朝西，
     * 靠近 -Y 时朝东——倒车回程（车头 +Y）左光重新面向西墙，
     * 保守限速自动恢复。
     */
    {
        bool heading_north =
            cosf(Path_CommandYawDeg() * (PATH_PI / 180.0f)) >= 0.0f;
        bool left_faces_wall = PathMap_IsMirrored() ? !heading_north
                                                    : heading_north;

        left_gate = (path_diagnostics.left_distance_cm <
                     PATH_LASER_MAX_CM) || left_faces_wall;
    }
    if (path_diagnostics.left_laser_online && left_gate)
    {
        distance_m = (float)path_diagnostics.left_distance_cm * 0.01f;
        requested_speed_mps = (left_command > 0) ?
            (float)left_command * PATH_COMMAND_TO_MPS : 0.0f;
        /* 朝左光逼近恒为车体 -X 分量。 */
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
        if (left_command > 0)
        {
            limited = PathSafety_LimitAxisCommand(
                (int16_t)left_command,
                path_diagnostics.left_allowed_speed_mps,
                PATH_COMMAND_TO_MPS);
            if (limited != (int16_t)left_command)
            {
                path_diagnostics.left_speed_limited = true;
                left_command = limited;
            }
        }
        if (path_left_blocked && (left_command > 0))
        {
            path_diagnostics.left_speed_limited = true;
            left_command = 0;
        }
        if ((left_command >= 0) &&
            ((measured_speed_mps >
              (path_diagnostics.left_allowed_speed_mps +
               PATH_HARD_STOP_SPEED_MPS)) ||
             (path_left_blocked && (path_last_output_vx < 0))))
        {
            hard_stop = true;
        }
    }

    *vy = (int16_t)front_command;
    *vx = (int16_t)-left_command;
    return hard_stop;
}

static void Path_ApplyOutput(int16_t vx, int16_t vy, int16_t vz,
                             bool force_stop)
{
    if (force_stop)
    {
        Chassis_StopAll();
        vx = 0;
        vy = 0;
        vz = 0;
    }
    else if ((vx != path_last_output_vx) ||
             (vy != path_last_output_vy) ||
             (vz != path_last_output_z))
    {
        (void)Chassis_SetVelocity(vx, vy, vz);
    }

    path_last_output_vx = vx;
    path_last_output_vy = vy;
    path_last_output_z = vz;
    path_diagnostics.output_vx = vx;
    path_diagnostics.output_vy = vy;
    path_diagnostics.output_z = vz;
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
    path_remote_vz = 0;
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
    path_return_button_armed = true;
    path_yaw_was_ready = false;
    path_front_blocked = false;
    path_left_blocked = false;
    path_side_detection_done = false;
    path_mirrored_detected = false;
    path_return_lock_yaw_deg = PATH_MAP_LOCK_YAW_DEG;
    path_return_target_pending = false;
    path_home_map_x_m = 0.0f;
    path_home_map_y_m = 0.0f;
    PathMap_SetMirrored(false);
    PathMap_SetReturnMode(false);
    PathMap_SetFinalApproach(false);

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
    Path_WriteRemoteMailbox(raw_vx, raw_vy, *z,
                            (uint8_t)(six_buttons & 0x3FU),
                            now_ms, true);

    /* 原 LoRa 调用保留，但只能重复最近一次已通过 1 ms 安全层的输出。 */
    *vx = path_last_output_vx;
    *vy = path_last_output_vy;
    *z = path_last_output_z;
}

void Path_NotifyRemoteOffline(uint32_t now_ms)
{
    Path_WriteRemoteMailbox(0, 0, 0, 0U, now_ms, false);
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
    *z = path_last_output_z;
}

void Path_Run1ms(uint32_t now_ms)
{
    path_remote_snapshot_t remote;
    int16_t vx;
    int16_t vy;
    int16_t vz;
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
    automatic_stop = Path_ProcessModeButton(&remote);
    Path_UpdateYawZeroLock();
    Path_UpdateLaserData();
    automatic_stop = Path_UpdateOdometryAndRoute() || automatic_stop;
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
    /*
     * 肩键旋转：去程保持锁 0（z 恒 0）；回程模式放开肩键，允许
     * 驾驶员微调航向（CalcOmega 手动旋转优先并跟随记录目标角，
     * 本层不再回拉，见 Path_UpdateYawZeroLock）。
     */
    vz = (path_diagnostics.return_mode && remote.online) ? remote.vz : 0;

    /*
     * 转向期间（回程模式且 yaw 未对齐锁定航向）忽略平移命令，
     * 避免坐标系假设失效期间误动；旋转由 IMU yaw 闭环完成，
     * 肩键微调仍然可用。
     */
    if (path_diagnostics.return_mode &&
        !path_diagnostics.return_yaw_aligned)
    {
        vx = 0;
        vy = 0;
    }

    if (path_diagnostics.single_axis_enabled)
    {
        /*
         * 单轴约束的语义是"只沿地图 X/Y 轴走"，而 vx/vy 是车体系：
         * 用实测 yaw 旋到地图系清除非规划轴分量后再旋回。原实现
         * 直接清车体分量，在掉头未完全对齐（允差 20°）时会留下
         * sin(误差角) 的横向泄漏（20° 时约 34%）。
         */
        float map_cmd_x;
        float map_cmd_y;
        float body_cmd_x;
        float body_cmd_y;
        float command_yaw_deg = Path_CommandYawDeg();

        Path_RotateVector((float)vx, (float)vy, command_yaw_deg,
                          &map_cmd_x, &map_cmd_y);
        if (path_diagnostics.active_axis == PATH_MAP_AXIS_X)
        {
            map_cmd_y = 0.0f;
        }
        else
        {
            map_cmd_x = 0.0f;
        }
        Path_RotateVector(map_cmd_x, map_cmd_y, -command_yaw_deg,
                          &body_cmd_x, &body_cmd_y);
        vx = (int16_t)((body_cmd_x >= 0.0f) ? (body_cmd_x + 0.5f)
                                            : (body_cmd_x - 0.5f));
        vy = (int16_t)((body_cmd_y >= 0.0f) ? (body_cmd_y + 0.5f)
                                            : (body_cmd_y - 0.5f));
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
                Path_ApplyOutput(0, 0, 0, true);
            }
            if (primask == 0U)
            {
                __enable_irq();
            }
            return;
        }
        Path_ApplyOutput(vx, vy, vz, automatic_stop || hard_stop);
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
