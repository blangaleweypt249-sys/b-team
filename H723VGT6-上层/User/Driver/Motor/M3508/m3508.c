/**
 * @file m3508.c
 * @brief 实现 DJI M3508 电机反馈解析、闭环控制和参数整定。
 */

#include "m3508.h"

#include <float.h>
#include <math.h>
#include <string.h>

#include "motor_online_tune.h"

#define M3508_ENCODER_COUNTS         8192U /**< 机械臂 M3508 输出轴转子编码器每圈的计数值。 */
#define M3508_REDUCTION_RATIO        (3591.0f / 187.0f) /**< 机械臂 M3508 输出轴转子到机构输出轴的减速比。 */
#define M3508_CURRENT_RAW_MAX        16384 /**< 机械臂 M3508 输出轴电流命令的协议原始值满量程。 */
#define M3508_CURRENT_MAX_A          20.0f /**< 机械臂 M3508 输出轴协议满量程对应的电流，单位：安培。 */
#define M3508_TWO_PI                 6.28318530718f /**< 角度换算使用的二倍圆周率数值。 */
#define M3508_QUINTIC_MAX_SPEED      1.875f /**< 五次平滑插值曲线的一阶导数最大系数，用于根据行程和速度计算轨迹时长。 */
#define M3508_QUINTIC_MAX_ACCEL      5.7735026919f /**< 五次平滑插值曲线的二阶导数最大系数，用于根据行程和加速度计算轨迹时长。 */
#define M3508_MIN_TRAJECTORY_S       0.01f /**< 机械臂 M3508 输出轴平滑位置轨迹允许使用的最短时间，单位：秒。 */
#define M3508_AUTOTUNE_TIMEOUT_MS    15000U /**< M3508 速度环自动整定允许持续的最长时间，单位：毫秒。 */
#define M3508_AUTOTUNE_CYCLES        5U /**< 机械臂 M3508 输出轴速度环自动整定需要采集的完整振荡周期数。 */
#define M3508_ZERO_STABLE_FRAMES     5U /**< 机械臂 M3508 输出轴建立软件零点前编码器必须连续稳定的反馈帧数。 */
#define M3508_ZERO_MAX_STEP_COUNTS   128 /**< 机械臂 M3508 输出轴置零期间相邻反馈仍可视为静止的最大编码器计数差。 */
#define M3508_ZERO_MAX_SPEED_RPM     30 /**< 机械臂 M3508 输出轴允许建立软件零点的最大转子速度，单位：转每分。 */

/** 保存 M3508 运行过程中需要集中管理的数据。 */
typedef struct
{
    m3508_pid_cfg_t cfg; /**< M3508 单个 PID 控制环的增益和限幅配置。 */
    float integral; /**< 当前积分累计值。 */
    float previous_error; /**< 上一次更新使用的误差。 */
    float full_integral_error; /**< PID 积分完全生效的误差范围。 */
    float integral_separation_error; /**< 超过该误差后停止继续积分的阈值。 */
    bool previous_valid; /**< PID 控制器是否保存了可用于微分计算的上次误差。 */
} m3508_pid_t;

/** 保存 M3508 运行过程中需要集中管理的数据。 */
typedef struct
{
    m3508_autotune_state_t state; /**< M3508 速度环自动整定的对外状态。 */
    float safety_velocity_rad_s; /**< 自动整定允许的安全速度上限，单位：弧度每秒。 */
    float phase_max_rad_s; /**< 当前继电整定半周期测得的最高速度，单位：弧度每秒。 */
    float phase_min_rad_s; /**< 当前继电整定半周期测得的最低速度，单位：弧度每秒。 */
    float positive_peak_rad_s; /**< 最近一个正向半周期记录的速度峰值，单位：弧度每秒。 */
    float period_sum_s; /**< 已完成振荡周期的总时间，单位：秒。 */
    float amplitude_sum_rad_s; /**< 已完成振荡周期的速度振幅总和，单位：弧度每秒。 */
    uint32_t started_ms; /**< 自动整定开始的系统毫秒时刻。 */
    uint32_t last_positive_cross_ms; /**< 速度最近一次正向跨越迟滞区的系统毫秒时刻。 */
    bool relay_positive; /**< 继电整定当前是否施加正向测试电流。 */
    bool positive_peak_valid; /**< 当前是否已记录有效正向速度峰值。 */
    bool positive_cross_valid; /**< 当前是否已记录一次有效正向过零时刻。 */
    uint8_t completed_cycles; /**< 自动整定已经完成的完整振荡周期数。 */
} m3508_autotune_t;

/** 保存 M3508 运行过程中需要集中管理的数据。 */
typedef struct
{
    volatile uint32_t feedback_sequence; /**< 反馈快照每次更新时递增的序号，用于无锁一致性读取。 */
    volatile bool feedback_valid; /**< 当前反馈快照是否已经由有效帧更新。 */
    volatile m3508_feedback_t feedback; /**< 最近一次有效电机反馈的快照。 */
    uint16_t previous_encoder; /**< 上一帧反馈的单圈转子编码器值。 */
    uint8_t zero_stable_frames; /**< 置零前编码器连续保持稳定的反馈帧数。 */
    int64_t zero_encoder_counts; /**< 建立软件零点时记录的累计转子计数。 */
    m3508_mode_t mode; /**< 当前采用的电机控制或调试工作模式。 */
    float target; /**< 当前控制模式下的电流、速度或位置目标值。 */
    float current_limit_a; /**< M3508的电流上限，单位：安培。 */
    float position_vel_limit_rad_s; /**< 位置轨迹允许的最大输出轴速度，单位：弧度每秒。 */
    float acceleration_limit_rad_s2; /**< M3508的轨迹加速度，单位：弧度每二次方秒。 */
    float speed_ramp_start_rad_s; /**< M3508 速度斜坡启动时的参考速度，单位：弧度每秒。 */
    float trajectory_start_rad; /**< 当前平滑轨迹的起始输出轴位置，单位：弧度。 */
    float trajectory_delta_rad; /**< 当前轨迹从起点到目标的输出轴位移，单位：弧度。 */
    float trajectory_duration_s; /**< 当前轨迹计划执行的总时间，单位：秒。 */
    float trajectory_elapsed_s; /**< 当前轨迹已经执行的时间，单位：秒。 */
    bool speed_reference_valid; /**< M3508 速度斜坡参考值是否已经初始化。 */
    bool trajectory_pending; /**< 位置目标改变后是否等待创建新的平滑轨迹。 */
    uint32_t command_updated_at_ms; /**< 最近一次更新电机目标的系统毫秒时刻。 */
    uint32_t feedback_monitor_started_at_ms; /**< 开始执行反馈超时监测的系统毫秒时刻。 */
    volatile m3508_timeout_stats_t timeout_stats; /**< 当前电机累计的命令和反馈超时状态。 */
    m3508_pid_t speed_pid; /**< M3508 速度环 PID 运行状态。 */
    m3508_pid_cfg_t speed_pid_base; /**< M3508 速度环在线调参前的基础 PID 配置。 */
    motor_online_pid_t speed_online_pid; /**< M3508 速度环在线 PID 调参状态。 */
    m3508_pid_t position_pid; /**< M3508 位置环 PID 运行状态。 */
    m3508_motion_state_t motion; /**< 当前速度斜坡和位置轨迹的诊断快照。 */
    m3508_autotune_t auto_tune; /**< 速度环继电自动整定的内部运行状态。 */
} m3508_context_t;

static m3508_cfg_t m3508_cfg;
static m3508_context_t
    m3508_context[M3508_CAN_BUS_COUNT][M3508_MOTOR_COUNT];

/* 功能：判断浮点数是否有限；用途：过滤 M3508 配置和目标中的异常值；返回 true 表示数值可用。 */
static bool M3508_IsFinite(float value /**< 待检查或限幅的 M3508 控制量 */)
{
    return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX);
}

/* 功能：检查 M3508 的 CAN 总线号和节点号；用途：保护上下文数组索引与分组路由；返回 true 表示地址合法。 */
static bool M3508_IsValidAddress(uint8_t can_bus /**< CAN 总线编号 */, uint8_t motor_id /**< DJI 电机编号 */)
{
    return (can_bus >= 1U) && (can_bus <= M3508_CAN_BUS_COUNT) &&
           (motor_id >= 1U) && (motor_id <= M3508_MOTOR_COUNT);
}

/* 功能：校验 M3508 PID 增益及限幅参数；用途：防止非法控制参数进入闭环；返回 true 表示配置可用。 */
static bool M3508_IsValidPid(const m3508_pid_cfg_t *cfg /**< 待设置或校验的 M3508 PID 配置 */)
{
    return (cfg != NULL) && M3508_IsFinite(cfg->kp) &&
           M3508_IsFinite(cfg->ki) && M3508_IsFinite(cfg->kd) &&
           M3508_IsFinite(cfg->integral_limit) &&
           M3508_IsFinite(cfg->output_limit) && (cfg->kp >= 0.0f) &&
           (cfg->ki >= 0.0f) && (cfg->kd >= 0.0f) &&
           (cfg->integral_limit >= 0.0f) && (cfg->output_limit > 0.0f);
}

/* 功能：按总线和节点定位 M3508 运行上下文；用途：访问目标、反馈、PID 和统计状态；返回 NULL 表示地址无效。 */
static m3508_context_t *M3508_GetContext(uint8_t can_bus /**< CAN 总线编号 */,
                                         uint8_t motor_id /**< DJI 电机编号 */)
{
    if (!M3508_IsValidAddress(can_bus, motor_id))
    {
        return NULL;
    }
    return &m3508_context[can_bus - 1U][motor_id - 1U];
}

/* 功能：把浮点数限制在给定区间；用途：约束积分、电流、速度和轨迹量；返回值表示限幅结果。 */
static float M3508_Clamp(float value /**< 待检查或限幅的 M3508 控制量 */, float min /**< 允许输出的下限 */, float max /**< 允许输出的上限 */)
{
    if (value < min)
    {
        return min;
    }
    if (value > max)
    {
        return max;
    }
    return value;
}

/* 功能：读取大端 16 位无符号整数；用途：解析 DJI M3508 反馈字段；返回值表示解码结果。 */
static uint16_t M3508_ReadU16Be(const uint8_t *data /**< M3508反馈帧中大端16位字段的首地址 */)
{
    return ((uint16_t)data[0] << 8U) | data[1];
}

/* 功能：计算跨越编码器零点后的最短计数差；用途：展开多圈转子位置；返回值表示带方向的增量计数。 */
static int32_t M3508_EncoderDelta(uint16_t encoder /**< 当前反馈的单圈编码器原始值 */,
                                  uint16_t previous_encoder /**< 上一帧反馈的单圈编码器原始值 */)
{
    int32_t delta;

    delta = (int32_t)encoder - (int32_t)previous_encoder;
    if (delta > ((int32_t)M3508_ENCODER_COUNTS / 2))
    {
        delta -= (int32_t)M3508_ENCODER_COUNTS;
    }
    else if (delta < -((int32_t)M3508_ENCODER_COUNTS / 2))
    {
        delta += (int32_t)M3508_ENCODER_COUNTS;
    }
    return delta;
}

/* 功能：清空 M3508 单个 PID 的积分和历史误差；用途：模式切换或停机后重新起算；无返回值表示运行状态已复位。 */
static void M3508_ResetPid(m3508_pid_t *pid /**< 需要操作的 PID 控制器 */)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->previous_valid = false;
}

/* 功能：执行一次带积分和输出限幅的 PID 计算；用途：实现位置环或基础速度环；返回值表示限幅后的控制量。 */
static float M3508_PidCalc(m3508_pid_t *pid /**< 需要操作的 PID 控制器 */,
                           float error /**< 当前控制误差 */,
                           float dt_s /**< 本次计算的控制周期，单位：秒 */,
                           float output_limit /**< PID 控制器输出的绝对值上限 */)
{
    float derivative;
    float previous_integral;
    float integration_weight;
    float absolute_error;
    float candidate_integral;
    float unsaturated_output;
    float output;

    derivative = 0.0f;
    previous_integral = pid->integral;
    integration_weight = 1.0f;
    absolute_error = (error < 0.0f) ? -error : error;
    if (pid->previous_valid)
    {
        derivative = (error - pid->previous_error) / dt_s;
    }

    if ((pid->cfg.ki <= 0.0f) || (pid->cfg.integral_limit <= 0.0f))
    {
        candidate_integral = 0.0f;
    }
    else if ((pid->integral_separation_error > 0.0f) &&
             (absolute_error >= pid->integral_separation_error))
    {
        integration_weight = 0.0f;
        candidate_integral = 0.0f;
    }
    else
    {
        if ((pid->integral_separation_error >
             pid->full_integral_error) &&
            (absolute_error > pid->full_integral_error))
        {
            integration_weight =
                (pid->integral_separation_error - absolute_error) /
                (pid->integral_separation_error -
                 pid->full_integral_error);
        }
        candidate_integral = M3508_Clamp(
            previous_integral + integration_weight * error * dt_s,
            -pid->cfg.integral_limit,
            pid->cfg.integral_limit);
    }

    if (output_limit > pid->cfg.output_limit)
    {
        output_limit = pid->cfg.output_limit;
    }
    unsaturated_output = pid->cfg.kp * error +
                         pid->cfg.ki * candidate_integral +
                         pid->cfg.kd * derivative;
    output = M3508_Clamp(unsaturated_output, -output_limit, output_limit);
    if ((integration_weight > 0.0f) &&
        (unsaturated_output != output) &&
        ((error * unsaturated_output) > 0.0f))
    {
        candidate_integral = previous_integral;
        unsaturated_output = pid->cfg.kp * error +
                             pid->cfg.ki * candidate_integral +
                             pid->cfg.kd * derivative;
        output = M3508_Clamp(unsaturated_output,
                             -output_limit,
                             output_limit);
    }
    pid->integral = candidate_integral;
    pid->previous_error = error;
    pid->previous_valid = true;
    return output;
}

/* 功能：按当前速度 PID 配置在线调参器；用途：建立自适应增益边界和学习参数；返回 true 表示初始化成功。 */
static bool M3508_ConfigureOnlinePid(m3508_context_t *context /**< 需要更新的 M3508 驱动上下文 */,
                                     bool enabled /**< 是否启用 M3508 速度环在线 PID 调参 */)
{
    motor_online_pid_cfg_t cfg;
    const m3508_pid_cfg_t *base;

    base = &context->speed_pid_base;
    (void)memset(&cfg, 0, sizeof(cfg));
    cfg.strategy = MOTOR_ONLINE_STRATEGY_HYBRID;
    cfg.base_gains.kp = base->kp;
    cfg.base_gains.ki = base->ki;
    cfg.base_gains.kd = base->kd;
    cfg.minimum_gains.kp = base->kp * 0.55f;
    cfg.minimum_gains.ki = base->ki * 0.40f;
    cfg.minimum_gains.kd = base->kd * 0.40f;
    cfg.maximum_gains.kp = base->kp * 1.55f;
    cfg.maximum_gains.ki = base->ki * 1.60f;
    cfg.maximum_gains.kd = base->kd * 1.80f;
    cfg.learning_rate.kp = base->kp * 0.0005f;
    cfg.learning_rate.ki = base->ki * 0.00025f;
    cfg.learning_rate.kd = base->kd * 0.0002f;
    cfg.adaptation_deadband = 0.083776f;
    cfg.normalization_epsilon = 0.005236f;
    cfg.adaptation_leak_per_s = 0.10f;
    cfg.small_error_threshold = 0.314159f;
    cfg.large_error_threshold = 4.712389f;
    cfg.error_rate_threshold = 47.123890f;
    cfg.boost_factor = 1.18f;
    cfg.damping_factor = 0.68f;
    cfg.smoothing = 0.10f;
    return MotorOnlinePid_Init(&context->speed_online_pid, &cfg, enabled);
}

/* 功能：使用在线增益执行 M3508 速度环计算；用途：把目标转速转换为电流命令；返回值表示限幅后的电流安培值。 */
static float M3508_SpeedPidCalc(m3508_context_t *context /**< 需要更新的 M3508 驱动上下文 */,
                                float error /**< 当前控制误差 */,
                                float dt_s /**< 本次计算的控制周期，单位：秒 */,
                                float output_limit /**< PID 控制器输出的绝对值上限 */)
{
    motor_online_gains_t gains;

    gains = MotorOnlinePid_Update(&context->speed_online_pid, error, dt_s);
    context->speed_pid.cfg.kp = gains.kp;
    context->speed_pid.cfg.ki = gains.ki;
    context->speed_pid.cfg.kd = gains.kd;
    context->speed_pid.cfg.integral_limit =
        context->speed_pid_base.integral_limit;
    context->speed_pid.cfg.output_limit =
        context->speed_pid_base.output_limit;
    return M3508_PidCalc(&context->speed_pid,
                         error,
                         dt_s,
                         output_limit);
}

/* 功能：将安培值换算并限制为 M3508 原始电流字段；用途：填充 DJI 分组帧；返回值表示协议电流值。 */
static int16_t M3508_CurrentToRaw(float current_a /**< 目标或限制电流，单位：安培 */)
{
    float raw;

    current_a = M3508_Clamp(current_a,
                            -M3508_CURRENT_MAX_A,
                            M3508_CURRENT_MAX_A);
    raw = current_a * (float)M3508_CURRENT_RAW_MAX / M3508_CURRENT_MAX_A;
    if (raw >= 0.0f)
    {
        return (int16_t)(raw + 0.5f);
    }
    return (int16_t)(raw - 0.5f);
}

/* 功能：复位 M3508 双环 PID、轨迹、斜坡和在线调参状态；用途：停机或控制模式改变时清除历史；无返回值表示状态已归零。 */
static void M3508_ResetControl(m3508_context_t *context /**< 需要更新的 M3508 驱动上下文 */)
{
    M3508_ResetPid(&context->speed_pid);
    MotorOnlinePid_Reset(&context->speed_online_pid, true);
    context->speed_pid.cfg = context->speed_pid_base;
    M3508_ResetPid(&context->position_pid);
    context->speed_reference_valid = false;
    context->trajectory_pending = false;
    context->motion.trajectory_active = false;
    context->motion.trajectory_progress = 1.0f;
    context->motion.reference_acceleration_rad_s2 = 0.0f;
}

/* 功能：以当前反馈为起点建立新的位置轨迹；用途：让位置目标按限速和加速度平滑变化；无返回值表示轨迹状态已初始化。 */
static void M3508_BeginPositionTrajectory(m3508_context_t *context /**< 需要更新的 M3508 驱动上下文 */,
                                           float position_rad /**< 新轨迹起点的实测位置，单位：弧度 */)
{
    float distance_rad;
    float speed_duration_s;
    float acceleration_duration_s;

    distance_rad = context->target - position_rad;
    context->trajectory_start_rad = position_rad;
    context->trajectory_delta_rad = distance_rad;
    speed_duration_s = M3508_QUINTIC_MAX_SPEED * fabsf(distance_rad) /
                       context->position_vel_limit_rad_s;
    acceleration_duration_s = sqrtf(M3508_QUINTIC_MAX_ACCEL *
                                    fabsf(distance_rad) /
                                    context->acceleration_limit_rad_s2);
    context->trajectory_duration_s =
        (speed_duration_s > acceleration_duration_s) ?
        speed_duration_s : acceleration_duration_s;
    if (context->trajectory_duration_s < M3508_MIN_TRAJECTORY_S)
    {
        context->trajectory_duration_s = M3508_MIN_TRAJECTORY_S;
    }
    context->trajectory_elapsed_s = 0.0f;
    context->trajectory_pending = false;
    context->motion.reference_position_rad = position_rad;
    context->motion.reference_velocity_rad_s = 0.0f;
    context->motion.reference_acceleration_rad_s2 = 0.0f;
    context->motion.trajectory_progress = 0.0f;
    context->motion.trajectory_active = fabsf(distance_rad) > 0.000001f;
}

/* 功能：推进一个周期的位置轨迹位置和速度；用途：生成受限的内部位置给定；无返回值表示轨迹状态已更新。 */
static void M3508_UpdatePositionTrajectory(m3508_context_t *context /**< 需要更新的 M3508 驱动上下文 */,
                                            float dt_s /**< 本次计算的控制周期，单位：秒 */)
{
    float tau;
    float tau2;
    float tau3;
    float tau4;
    float tau5;
    float position_scale;
    float velocity_scale;
    float acceleration_scale;

    if (!context->motion.trajectory_active)
    {
        context->motion.reference_position_rad = context->target;
        context->motion.reference_velocity_rad_s = 0.0f;
        context->motion.reference_acceleration_rad_s2 = 0.0f;
        context->motion.trajectory_progress = 1.0f;
        return;
    }

    tau = context->trajectory_elapsed_s / context->trajectory_duration_s;
    if (tau >= 1.0f)
    {
        context->motion.reference_position_rad = context->target;
        context->motion.reference_velocity_rad_s = 0.0f;
        context->motion.reference_acceleration_rad_s2 = 0.0f;
        context->motion.trajectory_progress = 1.0f;
        context->motion.trajectory_active = false;
        return;
    }

    tau2 = tau * tau;
    tau3 = tau2 * tau;
    tau4 = tau3 * tau;
    tau5 = tau4 * tau;
    position_scale = 10.0f * tau3 - 15.0f * tau4 + 6.0f * tau5;
    velocity_scale = 30.0f * tau2 - 60.0f * tau3 + 30.0f * tau4;
    acceleration_scale = 60.0f * tau - 180.0f * tau2 + 120.0f * tau3;
    context->motion.reference_position_rad =
        context->trajectory_start_rad +
        context->trajectory_delta_rad * position_scale;
    context->motion.reference_velocity_rad_s =
        context->trajectory_delta_rad * velocity_scale /
        context->trajectory_duration_s;
    context->motion.reference_acceleration_rad_s2 =
        context->trajectory_delta_rad * acceleration_scale /
        (context->trajectory_duration_s * context->trajectory_duration_s);
    context->motion.trajectory_progress = tau;
    context->trajectory_elapsed_s += dt_s;
}

/* 功能：按加速度限制推进速度给定；用途：避免速度命令阶跃；无返回值表示内部参考速度已更新。 */
static void M3508_UpdateSpeedRamp(m3508_context_t *context /**< 需要更新的 M3508 驱动上下文 */,
                                  float actual_velocity_rad_s /**< J4310 当前实测关节速度，单位：弧度每秒 */,
                                  float dt_s /**< 本次计算的控制周期，单位：秒 */)
{
    float delta_rad_s;
    float step_rad_s;
    float total_delta_rad_s;

    if (!context->speed_reference_valid)
    {
        context->motion.reference_velocity_rad_s = actual_velocity_rad_s;
        context->speed_ramp_start_rad_s = actual_velocity_rad_s;
        context->speed_reference_valid = true;
    }
    delta_rad_s = context->target - context->motion.reference_velocity_rad_s;
    step_rad_s = M3508_Clamp(delta_rad_s,
                             -context->acceleration_limit_rad_s2 * dt_s,
                             context->acceleration_limit_rad_s2 * dt_s);
    context->motion.reference_velocity_rad_s += step_rad_s;
    context->motion.reference_acceleration_rad_s2 = step_rad_s / dt_s;
    total_delta_rad_s = context->target - context->speed_ramp_start_rad_s;
    if (fabsf(total_delta_rad_s) <= 0.000001f)
    {
        context->motion.trajectory_progress = 1.0f;
    }
    else
    {
        context->motion.trajectory_progress = M3508_Clamp(
            1.0f - fabsf(context->target -
                         context->motion.reference_velocity_rad_s) /
                   fabsf(total_delta_rad_s),
            0.0f,
            1.0f);
    }
    context->motion.trajectory_active =
        fabsf(context->target - context->motion.reference_velocity_rad_s) >
        0.0001f;
}

/* 功能：将 M3508 自动整定标记为失败并清理测试输出；用途：统一处理超时或非法工况；无返回值表示整定已终止。 */
static void M3508_FailAutoTune(m3508_context_t *context /**< 需要更新的 M3508 驱动上下文 */)
{
    context->auto_tune.state.status = M3508_AUTOTUNE_FAILED;
    context->mode = M3508_MODE_STOP;
    context->target = 0.0f;
    context->motion.current_command_a = 0.0f;
}

/* 功能：执行 M3508 速度环自动整定的一步状态机；用途：施加测试激励并估算 PID；返回值表示本周期测试电流。 */
static float M3508_AutoTuneStep(m3508_context_t *context /**< 需要更新的 M3508 驱动上下文 */,
                                float velocity_rad_s /**< M3508 当前实测速度，单位：弧度每秒 */,
                                uint32_t tick_ms /**< 当前系统毫秒时刻 */)
{
    m3508_autotune_t *tune;

    tune = &context->auto_tune;
    if (((tick_ms - tune->started_ms) > M3508_AUTOTUNE_TIMEOUT_MS) ||
        (fabsf(velocity_rad_s) > tune->safety_velocity_rad_s))
    {
        M3508_FailAutoTune(context);
        return 0.0f;
    }
    if (velocity_rad_s > tune->phase_max_rad_s)
    {
        tune->phase_max_rad_s = velocity_rad_s;
    }
    if (velocity_rad_s < tune->phase_min_rad_s)
    {
        tune->phase_min_rad_s = velocity_rad_s;
    }
    if (tune->relay_positive &&
        (velocity_rad_s >= tune->state.hysteresis_rad_s))
    {
        tune->relay_positive = false;
        if (tune->positive_cross_valid && tune->positive_peak_valid)
        {
            float period_s;
            float amplitude_rad_s;

            period_s = (float)(tick_ms - tune->last_positive_cross_ms) /
                       1000.0f;
            amplitude_rad_s =
                (tune->positive_peak_rad_s - tune->phase_min_rad_s) * 0.5f;
            if ((period_s > 0.0f) && (amplitude_rad_s > 0.0f))
            {
                tune->period_sum_s += period_s;
                tune->amplitude_sum_rad_s += amplitude_rad_s;
                tune->completed_cycles++;
            }
        }
        tune->last_positive_cross_ms = tick_ms;
        tune->positive_cross_valid = true;
        tune->phase_max_rad_s = velocity_rad_s;
        tune->phase_min_rad_s = velocity_rad_s;
    }
    else if (!tune->relay_positive &&
             (velocity_rad_s <= -tune->state.hysteresis_rad_s))
    {
        tune->relay_positive = true;
        tune->positive_peak_rad_s = tune->phase_max_rad_s;
        tune->positive_peak_valid = true;
        tune->phase_max_rad_s = velocity_rad_s;
        tune->phase_min_rad_s = velocity_rad_s;
    }

    if (tune->completed_cycles >= M3508_AUTOTUNE_CYCLES)
    {
        float period_s;
        float amplitude_rad_s;
        float ultimate_gain;

        period_s = tune->period_sum_s / (float)tune->completed_cycles;
        amplitude_rad_s = tune->amplitude_sum_rad_s /
                          (float)tune->completed_cycles;
        ultimate_gain = 4.0f * tune->state.relay_current_a /
                        (0.5f * M3508_TWO_PI * amplitude_rad_s);
        tune->state.oscillation_amplitude_rad_s = amplitude_rad_s;
        tune->state.ultimate_period_s = period_s;
        tune->state.ultimate_gain = ultimate_gain;
        tune->state.result = context->speed_pid_base;
        tune->state.result.kp = ultimate_gain / 3.2f;
        tune->state.result.ki = tune->state.result.kp / (2.2f * period_s);
        tune->state.result.kd = 0.0f;
        context->speed_pid_base = tune->state.result;
        context->speed_pid.cfg = tune->state.result;
        (void)M3508_ConfigureOnlinePid(
            context, context->speed_online_pid.enabled != 0U);
        M3508_ResetPid(&context->speed_pid);
        tune->state.status = M3508_AUTOTUNE_COMPLETE;
        context->mode = M3508_MODE_STOP;
        context->target = 0.0f;
        context->motion.current_command_a = 0.0f;
        return 0.0f;
    }

    context->motion.reference_velocity_rad_s = 0.0f;
    context->motion.reference_acceleration_rad_s2 = 0.0f;
    context->motion.velocity_error_rad_s = -velocity_rad_s;
    context->motion.current_command_a = tune->relay_positive ?
                                        tune->state.relay_current_a :
                                        -tune->state.relay_current_a;
    return context->motion.current_command_a;
}

/* 功能：根据反馈新鲜度更新 M3508 超时统计；用途：累计连续丢帧、恢复和最大间隔；无返回值表示统计已刷新。 */
static void M3508_UpdateTimeoutStats(m3508_context_t *context /**< 需要更新的 M3508 驱动上下文 */,
                                     bool feedback_valid /**< 当前反馈快照是否有效 */,
                                     const m3508_feedback_t *feedback /**< 本周期用于更新 M3508 超时统计的反馈快照 */,
                                     uint32_t tick_ms /**< 当前系统毫秒时刻 */)
{
    bool command_timed_out;
    bool feedback_timed_out;

    command_timed_out = false;
    feedback_timed_out = false;
    if ((context->mode != M3508_MODE_STOP) &&
        (m3508_cfg.command_timeout_ms > 0U))
    {
        command_timed_out =
            (tick_ms - context->command_updated_at_ms) >
            m3508_cfg.command_timeout_ms;
    }
    if ((context->mode != M3508_MODE_STOP) &&
        (m3508_cfg.feedback_timeout_ms > 0U))
    {
        if (feedback_valid)
        {
            feedback_timed_out =
                (tick_ms - feedback->updated_at_ms) >
                m3508_cfg.feedback_timeout_ms;
        }
        else
        {
            feedback_timed_out =
                (tick_ms - context->feedback_monitor_started_at_ms) >
                m3508_cfg.feedback_timeout_ms;
        }
    }

    if (command_timed_out && !context->timeout_stats.command_timed_out)
    {
        context->timeout_stats.command_timeout_count++;
    }
    if (feedback_timed_out && !context->timeout_stats.feedback_timed_out)
    {
        context->timeout_stats.feedback_timeout_count++;
    }
    context->timeout_stats.command_timed_out = command_timed_out;
    context->timeout_stats.feedback_timed_out = feedback_timed_out;
}

/* 功能：初始化全部 M3508 上下文和默认控制参数；用途：建立各 CAN 总线的反馈与双环控制状态；返回 true 表示配置被接受。 */
bool M3508_Init(const m3508_cfg_t *cfg /**< M3508 电流限幅、轨迹及超时配置 */)
{
    uint32_t bus_index;
    uint32_t motor_index;

    if ((cfg == NULL) || !M3508_IsFinite(cfg->current_limit_a) ||
        !M3508_IsFinite(cfg->position_vel_limit_rad_s) ||
        !M3508_IsFinite(cfg->acceleration_limit_rad_s2) ||
        (cfg->current_limit_a <= 0.0f) ||
        (cfg->current_limit_a > M3508_CURRENT_MAX_A) ||
        (cfg->position_vel_limit_rad_s <= 0.0f) ||
        (cfg->acceleration_limit_rad_s2 <= 0.0f) ||
        !M3508_IsValidPid(&cfg->speed_pid) ||
        !M3508_IsValidPid(&cfg->position_pid))
    {
        return false;
    }

    m3508_cfg = *cfg;
    (void)memset(m3508_context, 0, sizeof(m3508_context));
    for (bus_index = 0U; bus_index < M3508_CAN_BUS_COUNT; bus_index++)
    {
        for (motor_index = 0U; motor_index < M3508_MOTOR_COUNT;
             motor_index++)
        {
            m3508_context_t *context;

            context = &m3508_context[bus_index][motor_index];
            context->mode = M3508_MODE_STOP;
            context->current_limit_a = cfg->current_limit_a;
            context->position_vel_limit_rad_s =
                cfg->position_vel_limit_rad_s;
            context->acceleration_limit_rad_s2 =
                cfg->acceleration_limit_rad_s2;
            context->motion.trajectory_progress = 1.0f;
            context->auto_tune.state.status = M3508_AUTOTUNE_IDLE;
            context->speed_pid.cfg = cfg->speed_pid;
            context->speed_pid.full_integral_error = 0.314159f;
            context->speed_pid.integral_separation_error = 4.712389f;
            context->speed_pid_base = cfg->speed_pid;
            if (!M3508_ConfigureOnlinePid(context, false))
            {
                return false;
            }
            context->position_pid.cfg = cfg->position_pid;
            context->position_pid.full_integral_error = 0.01f;
            context->position_pid.integral_separation_error = 0.25f;
            context->feedback.can_bus = (uint8_t)(bus_index + 1U);
            context->feedback.motor_id = (uint8_t)(motor_index + 1U);
        }
    }
    return true;
}

/* 功能：设置指定 M3508 的停止、电流、速度或位置目标；用途：更新下一控制周期的命令；返回 true 表示模式和目标合法。 */
bool M3508_SetTarget(uint8_t can_bus /**< CAN 总线编号 */,
                     uint8_t motor_id /**< DJI 电机编号 */,
                     m3508_mode_t mode /**< M3508 目标值采用的控制模式 */,
                     float target /**< 按 M3508 控制模式解释的电流、速度或位置目标 */,
                     uint32_t tick_ms /**< 当前系统毫秒时刻 */)
{
    m3508_context_t *context;
    bool mode_changed;
    bool target_changed;

    context = M3508_GetContext(can_bus, motor_id);
    if ((context == NULL) || !M3508_IsFinite(target) ||
        (mode > M3508_MODE_POSITION))
    {
        return false;
    }
    mode_changed = context->mode != mode;
    target_changed = mode_changed || (fabsf(context->target - target) > 0.000001f);
    if ((context->mode == M3508_MODE_STOP) &&
        (mode != M3508_MODE_STOP))
    {
        context->feedback_monitor_started_at_ms = tick_ms;
    }
    if (mode_changed)
    {
        M3508_ResetControl(context);
    }
    context->auto_tune.state.status = M3508_AUTOTUNE_IDLE;
    context->mode = mode;
    context->target = target;
    context->motion.final_position_rad =
        (mode == M3508_MODE_POSITION) ? target : 0.0f;
    context->motion.final_velocity_rad_s =
        (mode == M3508_MODE_VELOCITY) ? target : 0.0f;
    context->motion.reference_acceleration_rad_s2 = 0.0f;
    if (mode == M3508_MODE_POSITION)
    {
        if (target_changed)
        {
            context->trajectory_pending = true;
        }
        context->speed_reference_valid = false;
    }
    else if (mode == M3508_MODE_VELOCITY)
    {
        if (target_changed)
        {
            context->speed_ramp_start_rad_s =
                context->motion.reference_velocity_rad_s;
        }
        if (mode_changed)
        {
            context->speed_reference_valid = false;
        }
        context->trajectory_pending = false;
    }
    else
    {
        context->speed_reference_valid = false;
        context->trajectory_pending = false;
        context->motion.trajectory_active = false;
        context->motion.trajectory_progress = 1.0f;
    }
    context->command_updated_at_ms = tick_ms;
    return true;
}

/* 功能：读取指定 M3508 的最新反馈；用途：获取编码器、速度、电流和温度；返回 true 表示已收到有效帧。 */
bool M3508_GetFeedback(uint8_t can_bus /**< CAN 总线编号 */,
                       uint8_t motor_id /**< DJI 电机编号 */,
                       m3508_feedback_t *feedback /**< 用于写出最新 M3508 反馈的对象 */)
{
    const m3508_context_t *context;
    uint32_t before;
    uint32_t after;
    bool valid;

    context = M3508_GetContext(can_bus, motor_id);
    if ((context == NULL) || (feedback == NULL))
    {
        return false;
    }

    for (;;)
    {
        before = context->feedback_sequence;
        if ((before & 1U) != 0U)
        {
            continue;
        }
        valid = context->feedback_valid;
        *feedback = context->feedback;
        after = context->feedback_sequence;
        if (before == after)
        {
            break;
        }
    }
    return valid;
}

/* 功能：读取带零点和多圈位置的 M3508 反馈快照；用途：供诊断与位置显示使用；返回 true 表示快照有效。 */
bool M3508_GetFeedbackSnapshot(uint8_t can_bus /**< CAN 总线编号 */,
                               uint8_t motor_id /**< DJI 电机编号 */,
                               m3508_feedback_t *feedback /**< 用于写出最新 M3508 反馈的对象 */,
                               bool *zero_valid /**< 用于写出电机软件零点是否有效 */)
{
    const m3508_context_t *context;
    uint32_t before;
    uint32_t after;
    bool valid;

    context = M3508_GetContext(can_bus, motor_id);
    if ((context == NULL) || (feedback == NULL) || (zero_valid == NULL))
    {
        return false;
    }
    for (;;)
    {
        before = context->feedback_sequence;
        if ((before & 1U) != 0U)
        {
            continue;
        }
        valid = context->feedback_valid;
        *feedback = context->feedback;
        after = context->feedback_sequence;
        if (before == after)
        {
            break;
        }
    }
    *zero_valid = valid;
    return feedback->rx_frames > 0U;
}

/* 功能：读取指定 M3508 的通信超时统计；用途：诊断丢帧、恢复次数和帧间隔；返回 true 表示统计已写出。 */
bool M3508_GetTimeoutStats(uint8_t can_bus /**< CAN 总线编号 */,
                           uint8_t motor_id /**< DJI 电机编号 */,
                           m3508_timeout_stats_t *stats /**< 用于写出 M3508 命令与反馈超时统计的对象 */)
{
    const m3508_context_t *context;

    context = M3508_GetContext(can_bus, motor_id);
    if ((context == NULL) || (stats == NULL))
    {
        return false;
    }

    stats->command_timed_out = context->timeout_stats.command_timed_out;
    stats->feedback_timed_out = context->timeout_stats.feedback_timed_out;
    stats->command_timeout_count =
        context->timeout_stats.command_timeout_count;
    stats->feedback_timeout_count =
        context->timeout_stats.feedback_timeout_count;
    return true;
}

/* 功能：解析 M3508 DJI 反馈帧并展开位置；用途：更新闭环反馈、时间戳和通信统计；返回 true 表示帧地址有效。 */
bool M3508_OnFrame(uint8_t can_bus /**< CAN 总线编号 */,
                    uint8_t motor_id /**< DJI 电机编号 */,
                    const can_frame_t *frame /**< 待解析的 CAN 接收帧 */,
                    uint32_t tick_ms /**< 当前系统毫秒时刻 */)
{
    m3508_context_t *context;
    volatile m3508_feedback_t *feedback;
    uint16_t encoder;
    int16_t rotor_speed_rpm;
    int32_t delta;
    int64_t total_counts;
    int64_t relative_counts;
    uint32_t sequence;

    context = M3508_GetContext(can_bus, motor_id);
    if ((context == NULL) || (frame == NULL) || frame->extended ||
        (frame->dlc != 8U) ||
        (frame->id != (0x200U + motor_id)))
    {
        return false;
    }

    encoder = M3508_ReadU16Be(frame->data);
    rotor_speed_rpm = (int16_t)M3508_ReadU16Be(&frame->data[2]);
    if (encoder >= M3508_ENCODER_COUNTS)
    {
        return false;
    }

    feedback = &context->feedback;
    sequence = context->feedback_sequence;
    context->feedback_sequence = sequence + 1U;

    if (!context->feedback_valid)
    {
        if ((rotor_speed_rpm < -M3508_ZERO_MAX_SPEED_RPM) ||
            (rotor_speed_rpm > M3508_ZERO_MAX_SPEED_RPM))
        {
            context->zero_stable_frames = 0U;
        }
        else if (context->zero_stable_frames == 0U)
        {
            context->zero_stable_frames = 1U;
        }
        else
        {
            delta = M3508_EncoderDelta(encoder,
                                        context->previous_encoder);
            if ((delta >= -M3508_ZERO_MAX_STEP_COUNTS) &&
                (delta <= M3508_ZERO_MAX_STEP_COUNTS))
            {
                if (context->zero_stable_frames <
                    M3508_ZERO_STABLE_FRAMES)
                {
                    context->zero_stable_frames++;
                }
            }
            else
            {
                context->zero_stable_frames = 1U;
            }
        }
        total_counts = encoder;
        context->zero_encoder_counts = total_counts;
        relative_counts = 0;
    }
    else
    {
        delta = M3508_EncoderDelta(encoder,
                                    context->previous_encoder);
        total_counts = feedback->total_encoder_counts + delta;
        relative_counts = total_counts - context->zero_encoder_counts;
    }
    context->previous_encoder = encoder;

    feedback->can_bus = can_bus;
    feedback->motor_id = motor_id;
    feedback->rotor_encoder = encoder;
    feedback->rotor_speed_rpm = rotor_speed_rpm;
    feedback->torque_current_raw =
        (int16_t)M3508_ReadU16Be(&frame->data[4]);
    feedback->temperature_c = frame->data[6];
    feedback->total_encoder_counts = total_counts;
    feedback->zero_encoder_counts = context->zero_encoder_counts;
    feedback->output_pos_rad =
        (float)relative_counts * M3508_TWO_PI /
        ((float)M3508_ENCODER_COUNTS * M3508_REDUCTION_RATIO);
    feedback->output_vel_rad_s =
        (float)feedback->rotor_speed_rpm * M3508_TWO_PI /
        (60.0f * M3508_REDUCTION_RATIO);
    feedback->torque_current_a =
        (float)feedback->torque_current_raw * M3508_CURRENT_MAX_A /
        (float)M3508_CURRENT_RAW_MAX;
    feedback->updated_at_ms = tick_ms;
    feedback->rx_frames++;
    context->feedback_valid =
        context->zero_stable_frames >= M3508_ZERO_STABLE_FRAMES;
    context->feedback_sequence = sequence + 2U;
    return true;
}

/* 功能：根据当前模式和反馈计算 M3508 原始电流命令；用途：驱动双环控制、轨迹和自动整定；返回 true 表示成功写出电流。 */
bool M3508_CalcCurrentRaw(uint8_t can_bus /**< CAN 总线编号 */,
                          uint8_t motor_id /**< DJI 电机编号 */,
                          uint32_t tick_ms /**< 当前系统毫秒时刻 */,
                          int16_t *current_raw /**< DJI 协议中的电流命令原始值 */)
{
    m3508_context_t *context;
    m3508_feedback_t feedback;
    float current_a;
    bool feedback_valid;

    context = M3508_GetContext(can_bus, motor_id);
    if ((context == NULL) || (current_raw == NULL))
    {
        return false;
    }

    current_a = 0.0f;
    feedback_valid = M3508_GetFeedback(can_bus, motor_id, &feedback);
    M3508_UpdateTimeoutStats(context, feedback_valid, &feedback, tick_ms);
    if (context->auto_tune.state.status == M3508_AUTOTUNE_RUNNING)
    {
        if (!feedback_valid ||
            ((m3508_cfg.feedback_timeout_ms > 0U) &&
             ((tick_ms - feedback.updated_at_ms) >
              m3508_cfg.feedback_timeout_ms)))
        {
            M3508_FailAutoTune(context);
            *current_raw = 0;
            return true;
        }
        current_a = M3508_AutoTuneStep(context,
                                       feedback.output_vel_rad_s,
                                       tick_ms);
        *current_raw = M3508_CurrentToRaw(current_a);
        return true;
    }
    if ((context->mode != M3508_MODE_STOP) &&
        (!feedback_valid || context->timeout_stats.command_timed_out ||
         context->timeout_stats.feedback_timed_out))
    {
        M3508_ResetControl(context);
        context->trajectory_pending =
            context->mode == M3508_MODE_POSITION;
        context->motion.current_command_a = 0.0f;
        *current_raw = 0;
        return true;
    }
    if (context->mode == M3508_MODE_STOP)
    {
        if (feedback_valid)
        {
            context->motion.reference_position_rad =
                feedback.output_pos_rad;
        }
        context->motion.reference_velocity_rad_s = 0.0f;
        context->motion.reference_acceleration_rad_s2 = 0.0f;
        context->motion.current_command_a = 0.0f;
        context->motion.position_error_rad = 0.0f;
        context->motion.velocity_error_rad_s = 0.0f;
        context->motion.trajectory_active = false;
        context->motion.trajectory_progress = 1.0f;
        M3508_ResetControl(context);
    }
    else
    {
        switch (context->mode)
        {
        case M3508_MODE_CURRENT:
            current_a = context->target;
            context->motion.reference_position_rad = feedback.output_pos_rad;
            context->motion.reference_velocity_rad_s =
                feedback.output_vel_rad_s;
            context->motion.reference_acceleration_rad_s2 = 0.0f;
            context->motion.position_error_rad = 0.0f;
            context->motion.velocity_error_rad_s = 0.0f;
            break;

        case M3508_MODE_VELOCITY:
            if (feedback_valid)
            {
                M3508_UpdateSpeedRamp(context,
                                      feedback.output_vel_rad_s,
                                      0.001f);
                context->motion.reference_position_rad =
                    feedback.output_pos_rad;
                context->motion.position_error_rad = 0.0f;
                context->motion.velocity_error_rad_s =
                    context->motion.reference_velocity_rad_s -
                    feedback.output_vel_rad_s;
                current_a = M3508_SpeedPidCalc(
                    context,
                    context->motion.velocity_error_rad_s,
                    0.001f,
                    context->current_limit_a);
            }
            break;

        case M3508_MODE_POSITION:
        {
            float target_vel_rad_s;

            if (feedback_valid)
            {
                if (context->trajectory_pending)
                {
                    M3508_BeginPositionTrajectory(
                        context, feedback.output_pos_rad);
                }
                M3508_UpdatePositionTrajectory(context, 0.001f);
                context->motion.position_error_rad =
                    context->motion.reference_position_rad -
                    feedback.output_pos_rad;
                target_vel_rad_s = M3508_PidCalc(
                    &context->position_pid,
                    context->motion.position_error_rad,
                    0.001f,
                    context->position_vel_limit_rad_s);
                target_vel_rad_s = M3508_Clamp(
                    context->motion.reference_velocity_rad_s +
                    target_vel_rad_s,
                    -context->position_vel_limit_rad_s,
                    context->position_vel_limit_rad_s);
                context->motion.velocity_error_rad_s =
                    target_vel_rad_s - feedback.output_vel_rad_s;
                current_a = M3508_SpeedPidCalc(
                    context,
                    context->motion.velocity_error_rad_s,
                    0.001f,
                    context->current_limit_a);
            }
            break;
        }

        default:
            break;
        }
    }

    current_a = M3508_Clamp(current_a,
                            -context->current_limit_a,
                            context->current_limit_a);
    context->motion.current_command_a = current_a;
    *current_raw = M3508_CurrentToRaw(current_a);
    return true;
}

/* 功能：设置指定 M3508 的速度环 PID；用途：调整速度响应并重新配置在线调参；返回 true 表示参数有效。 */
bool M3508_SetSpeedPid(uint8_t can_bus /**< CAN 总线编号 */,
                       uint8_t motor_id /**< DJI 电机编号 */,
                       const m3508_pid_cfg_t *cfg /**< 待设置或校验的 M3508 PID 配置 */)
{
    m3508_context_t *context;
    bool enabled;

    context = M3508_GetContext(can_bus, motor_id);
    if ((context == NULL) || !M3508_IsValidPid(cfg))
    {
        return false;
    }
    enabled = context->speed_online_pid.enabled != 0U;
    context->speed_pid_base = *cfg;
    context->speed_pid.cfg = *cfg;
    if (!M3508_ConfigureOnlinePid(context, enabled))
    {
        return false;
    }
    M3508_ResetPid(&context->speed_pid);
    return true;
}

/* 功能：设置指定 M3508 的位置环 PID；用途：调整位置到速度的外环响应；返回 true 表示参数有效。 */
bool M3508_SetPositionPid(uint8_t can_bus /**< CAN 总线编号 */,
                          uint8_t motor_id /**< DJI 电机编号 */,
                          const m3508_pid_cfg_t *cfg /**< 待设置或校验的 M3508 PID 配置 */)
{
    m3508_context_t *context;

    context = M3508_GetContext(can_bus, motor_id);
    if ((context == NULL) || !M3508_IsValidPid(cfg))
    {
        return false;
    }
    context->position_pid.cfg = *cfg;
    M3508_ResetPid(&context->position_pid);
    return true;
}

/* 功能：设置指定 M3508 的软件电流限制；用途：约束所有闭环和直接控制输出；返回 true 表示限制合法。 */
bool M3508_SetCurrentLimit(uint8_t can_bus /**< CAN 总线编号 */,
                           uint8_t motor_id /**< DJI 电机编号 */,
                           float current_limit_a /**< 允许输出的最大电流，单位：安培 */)
{
    m3508_context_t *context;

    context = M3508_GetContext(can_bus, motor_id);
    if ((context == NULL) || !M3508_IsFinite(current_limit_a) ||
        (current_limit_a <= 0.0f) ||
        (current_limit_a > M3508_CURRENT_MAX_A))
    {
        return false;
    }
    context->current_limit_a = current_limit_a;
    return true;
}

/* 功能：设置 M3508 位置模式的最大速度；用途：限制位置轨迹运动速度；返回 true 表示限制合法。 */
bool M3508_SetPositionVelocityLimit(uint8_t can_bus /**< CAN 总线编号 */,
                                    uint8_t motor_id /**< DJI 电机编号 */,
                                    float velocity_limit_rad_s /**< 允许设置的速度上限，单位：弧度每秒 */)
{
    m3508_context_t *context;

    context = M3508_GetContext(can_bus, motor_id);
    if ((context == NULL) || !M3508_IsFinite(velocity_limit_rad_s) ||
        (velocity_limit_rad_s <= 0.0f))
    {
        return false;
    }
    context->position_vel_limit_rad_s = velocity_limit_rad_s;
    return true;
}

/* 功能：设置 M3508 速度变化的加速度限制；用途：平滑速度和位置轨迹；返回 true 表示限制合法。 */
bool M3508_SetAccelerationLimit(uint8_t can_bus /**< CAN 总线编号 */,
                                uint8_t motor_id /**< DJI 电机编号 */,
                                float acceleration_limit_rad_s2 /**< 允许设置的加速度上限，单位：弧度每二次方秒 */)
{
    m3508_context_t *context;

    context = M3508_GetContext(can_bus, motor_id);
    if ((context == NULL) || !M3508_IsFinite(acceleration_limit_rad_s2) ||
        (acceleration_limit_rad_s2 <= 0.0f))
    {
        return false;
    }
    context->acceleration_limit_rad_s2 = acceleration_limit_rad_s2;
    return true;
}

/* 功能：启用或关闭指定 M3508 的速度环在线调参；用途：切换固定与自适应 PID；返回 true 表示设置成功。 */
bool M3508_SetOnlinePidEnabled(uint8_t can_bus /**< CAN 总线编号 */,
                               uint8_t motor_id /**< DJI 电机编号 */,
                               bool enabled /**< 是否启用 M3508 速度环在线 PID 调参 */)
{
    m3508_context_t *context;

    context = M3508_GetContext(can_bus, motor_id);
    if (context == NULL)
    {
        return false;
    }
    MotorOnlinePid_SetEnabled(&context->speed_online_pid, enabled);
    context->speed_pid.cfg = context->speed_pid_base;
    M3508_ResetPid(&context->speed_pid);
    return true;
}

/* 功能：把当前 M3508 多圈位置记录为软件零点；用途：建立输出轴相对位置基准；返回 true 表示已有反馈且置零成功。 */
bool M3508_ZeroPosition(uint8_t can_bus /**< CAN 总线编号 */, uint8_t motor_id /**< DJI 电机编号 */)
{
    m3508_context_t *context;

    context = M3508_GetContext(can_bus, motor_id);
    if ((context == NULL) || !context->feedback_valid)
    {
        return false;
    }
    context->zero_encoder_counts = context->feedback.total_encoder_counts;
    context->feedback.zero_encoder_counts = context->zero_encoder_counts;
    context->feedback.output_pos_rad = 0.0f;
    context->motion.reference_position_rad = 0.0f;
    context->motion.final_position_rad = 0.0f;
    context->trajectory_pending = false;
    M3508_ResetPid(&context->position_pid);
    return true;
}

/* 功能：读取指定 M3508 的在线 PID 调参状态；用途：观察当前增益、误差和活动规则；返回 true 表示状态已写出。 */
bool M3508_GetOnlinePidState(uint8_t can_bus /**< CAN 总线编号 */,
                             uint8_t motor_id /**< DJI 电机编号 */,
                             m3508_online_pid_state_t *state /**< 用于写出 M3508 在线 PID 调参状态的对象 */)
{
    const m3508_context_t *context;

    context = M3508_GetContext(can_bus, motor_id);
    if ((context == NULL) || (state == NULL))
    {
        return false;
    }
    state->enabled = context->speed_online_pid.enabled != 0U;
    state->strategy = state->enabled ?
                      (uint8_t)MOTOR_ONLINE_STRATEGY_HYBRID : 0U;
    state->active_rule =
        (uint8_t)context->speed_online_pid.active_rule;
    state->applied_kp = context->speed_online_pid.applied_gains.kp;
    state->applied_ki = context->speed_online_pid.applied_gains.ki;
    state->applied_kd = context->speed_online_pid.applied_gains.kd;
    return true;
}

/* 功能：读取 M3508 内部斜坡和位置轨迹状态；用途：诊断当前平滑给定及轨迹是否活动；返回 true 表示状态有效。 */
bool M3508_GetMotionState(uint8_t can_bus /**< CAN 总线编号 */,
                          uint8_t motor_id /**< DJI 电机编号 */,
                          m3508_motion_state_t *state /**< 用于写出 M3508 斜坡与位置轨迹状态的对象 */)
{
    const m3508_context_t *context;

    context = M3508_GetContext(can_bus, motor_id);
    if ((context == NULL) || (state == NULL))
    {
        return false;
    }
    *state = context->motion;
    return true;
}

/* 功能：读取指定 M3508 的速度环 PID 配置；用途：显示或保存当前控制参数；返回 true 表示配置已写出。 */
bool M3508_GetSpeedPid(uint8_t can_bus /**< CAN 总线编号 */,
                       uint8_t motor_id /**< DJI 电机编号 */,
                       m3508_pid_cfg_t *cfg /**< 用于写出当前 M3508 PID 配置的对象 */)
{
    const m3508_context_t *context;

    context = M3508_GetContext(can_bus, motor_id);
    if ((context == NULL) || (cfg == NULL))
    {
        return false;
    }
    *cfg = context->speed_pid_base;
    return true;
}

/* 功能：读取指定 M3508 的位置环 PID 配置；用途：显示或保存当前控制参数；返回 true 表示配置已写出。 */
bool M3508_GetPositionPid(uint8_t can_bus /**< CAN 总线编号 */,
                          uint8_t motor_id /**< DJI 电机编号 */,
                          m3508_pid_cfg_t *cfg /**< 用于写出当前 M3508 PID 配置的对象 */)
{
    const m3508_context_t *context;

    context = M3508_GetContext(can_bus, motor_id);
    if ((context == NULL) || (cfg == NULL))
    {
        return false;
    }
    *cfg = context->position_pid.cfg;
    return true;
}

/* 功能：启动指定 M3508 的速度环自动整定；用途：通过继电反馈测试估算控制增益；返回 true 表示整定已进入运行态。 */
bool M3508_StartSpeedAutoTune(uint8_t can_bus /**< CAN 总线编号 */,
                              uint8_t motor_id /**< DJI 电机编号 */,
                              float relay_current_a /**< 自动整定施加的继电测试电流，单位：安培 */,
                              float hysteresis_rad_s /**< 自动整定切换电流方向的速度迟滞带，单位：弧度每秒 */,
                              float safety_velocity_rad_s /**< 自动整定允许达到的安全速度上限，单位：弧度每秒 */,
                              uint32_t tick_ms /**< 当前系统毫秒时刻 */)
{
    m3508_context_t *context;
    m3508_feedback_t feedback;

    context = M3508_GetContext(can_bus, motor_id);
    if ((context == NULL) || !M3508_IsFinite(relay_current_a) ||
        !M3508_IsFinite(hysteresis_rad_s) ||
        !M3508_IsFinite(safety_velocity_rad_s) ||
        (relay_current_a <= 0.0f) || (hysteresis_rad_s <= 0.0f) ||
        (safety_velocity_rad_s <= hysteresis_rad_s) ||
        !M3508_GetFeedback(can_bus, motor_id, &feedback) ||
        ((m3508_cfg.feedback_timeout_ms > 0U) &&
         ((tick_ms - feedback.updated_at_ms) >
          m3508_cfg.feedback_timeout_ms)) ||
        (relay_current_a > context->current_limit_a) ||
        (fabsf(feedback.output_vel_rad_s) >= safety_velocity_rad_s))
    {
        return false;
    }
    (void)memset(&context->auto_tune, 0, sizeof(context->auto_tune));
    context->auto_tune.state.status = M3508_AUTOTUNE_RUNNING;
    context->auto_tune.state.relay_current_a = relay_current_a;
    context->auto_tune.state.hysteresis_rad_s = hysteresis_rad_s;
    context->auto_tune.state.result = context->speed_pid_base;
    context->auto_tune.safety_velocity_rad_s = safety_velocity_rad_s;
    context->auto_tune.phase_max_rad_s = feedback.output_vel_rad_s;
    context->auto_tune.phase_min_rad_s = feedback.output_vel_rad_s;
    context->auto_tune.started_ms = tick_ms;
    context->auto_tune.relay_positive = true;
    context->mode = M3508_MODE_STOP;
    context->target = 0.0f;
    M3508_ResetControl(context);
    return true;
}

/* 功能：取消正在进行的 M3508 自动整定；用途：响应用户停止或安全条件变化；返回 true 表示地址有效并已取消。 */
bool M3508_CancelAutoTune(uint8_t can_bus /**< CAN 总线编号 */, uint8_t motor_id /**< DJI 电机编号 */)
{
    m3508_context_t *context;

    context = M3508_GetContext(can_bus, motor_id);
    if (context == NULL)
    {
        return false;
    }
    context->auto_tune.state.status = M3508_AUTOTUNE_IDLE;
    context->mode = M3508_MODE_STOP;
    context->target = 0.0f;
    context->motion.current_command_a = 0.0f;
    return true;
}

/* 功能：读取指定 M3508 的自动整定状态和结果；用途：向调试界面报告进度、参数或失败原因；返回 true 表示状态已写出。 */
bool M3508_GetAutoTuneState(uint8_t can_bus /**< CAN 总线编号 */,
                            uint8_t motor_id /**< DJI 电机编号 */,
                            m3508_autotune_state_t *state /**< 用于写出 M3508 自动整定状态和结果的对象 */)
{
    const m3508_context_t *context;

    context = M3508_GetContext(can_bus, motor_id);
    if ((context == NULL) || (state == NULL))
    {
        return false;
    }
    *state = context->auto_tune.state;
    return true;
}
