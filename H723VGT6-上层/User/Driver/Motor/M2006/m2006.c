#include "m2006.h"

#include <float.h>
#include <math.h>
#include <string.h>

#include "motor_online_tune.h"

#define M2006_ENCODER_COUNTS   8192U
#define M2006_REDUCTION_RATIO  36.0f
#define M2006_CURRENT_RAW_MAX  10000
#define M2006_CURRENT_MAX_A    10.0f
#define M2006_TWO_PI           6.28318530718f
#define M2006_QUINTIC_MAX_SPEED 1.875f
#define M2006_QUINTIC_MAX_ACCEL 5.7735026919f
#define M2006_MIN_TRAJECTORY_S  0.01f
#define M2006_AUTOTUNE_TIMEOUT_MS 15000U
#define M2006_AUTOTUNE_CYCLES   5U
#define M2006_ZERO_STABLE_FRAMES 5U
#define M2006_ZERO_MAX_STEP_COUNTS 128
#define M2006_ZERO_MAX_SPEED_RPM 30

typedef struct
{
    m2006_pid_cfg_t cfg;
    float integral;
    float previous_error;
    float full_integral_error;
    float integral_separation_error;
    bool previous_valid;
} m2006_pid_t;

typedef struct
{
    m2006_autotune_state_t state;
    float safety_velocity_rad_s;
    float phase_max_rad_s;
    float phase_min_rad_s;
    float positive_peak_rad_s;
    float period_sum_s;
    float amplitude_sum_rad_s;
    uint32_t started_ms;
    uint32_t last_positive_cross_ms;
    bool relay_positive;
    bool positive_peak_valid;
    bool positive_cross_valid;
    uint8_t completed_cycles;
} m2006_autotune_t;

typedef struct
{
    volatile uint32_t feedback_sequence;
    volatile bool feedback_valid;
    volatile m2006_feedback_t feedback;
    uint16_t previous_encoder;
    uint8_t zero_stable_frames;
    int64_t zero_encoder_counts;
    m2006_mode_t mode;
    float target;
    float current_limit_a;
    float position_vel_limit_rad_s;
    float acceleration_limit_rad_s2;
    float speed_ramp_start_rad_s;
    float trajectory_start_rad;
    float trajectory_delta_rad;
    float trajectory_duration_s;
    float trajectory_elapsed_s;
    bool speed_reference_valid;
    bool trajectory_pending;
    uint32_t command_updated_at_ms;
    uint32_t feedback_monitor_started_at_ms;
    volatile m2006_timeout_stats_t timeout_stats;
    m2006_pid_t speed_pid;
    m2006_pid_cfg_t speed_pid_base;
    motor_online_pid_t speed_online_pid;
    m2006_pid_t position_pid;
    m2006_motion_state_t motion;
    m2006_autotune_t auto_tune;
} m2006_context_t;

static m2006_cfg_t m2006_cfg;
static m2006_context_t
    m2006_context[M2006_CAN_BUS_COUNT][M2006_MOTOR_COUNT];

/* 功能：判断浮点数是否有限；用途：过滤 M2006 配置和目标中的异常值；返回 true 表示数值可用。 */
static bool M2006_IsFinite(float value)
{
    return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX);
}

/* 功能：检查 M2006 的 CAN 总线号和节点号；用途：保护上下文数组索引与分组路由；返回 true 表示地址合法。 */
static bool M2006_IsValidAddress(uint8_t can_bus, uint8_t motor_id)
{
    return (can_bus >= 1U) && (can_bus <= M2006_CAN_BUS_COUNT) &&
           (motor_id >= 1U) && (motor_id <= M2006_MOTOR_COUNT);
}

/* 功能：校验 M2006 PID 增益及限幅参数；用途：防止非法控制参数进入闭环；返回 true 表示配置可用。 */
static bool M2006_IsValidPid(const m2006_pid_cfg_t *cfg)
{
    return (cfg != NULL) && M2006_IsFinite(cfg->kp) &&
           M2006_IsFinite(cfg->ki) && M2006_IsFinite(cfg->kd) &&
           M2006_IsFinite(cfg->integral_limit) &&
           M2006_IsFinite(cfg->output_limit) && (cfg->kp >= 0.0f) &&
           (cfg->ki >= 0.0f) && (cfg->kd >= 0.0f) &&
           (cfg->integral_limit >= 0.0f) && (cfg->output_limit > 0.0f);
}

/* 功能：按总线和节点定位 M2006 运行上下文；用途：访问目标、反馈、PID 和统计状态；返回 NULL 表示地址无效。 */
static m2006_context_t *M2006_GetContext(uint8_t can_bus,
                                         uint8_t motor_id)
{
    if (!M2006_IsValidAddress(can_bus, motor_id))
    {
        return NULL;
    }
    return &m2006_context[can_bus - 1U][motor_id - 1U];
}

/* 功能：把浮点数限制在给定区间；用途：约束积分、电流、速度和轨迹量；返回值表示限幅结果。 */
static float M2006_Clamp(float value, float min, float max)
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

/* 功能：读取大端 16 位无符号整数；用途：解析 DJI M2006 反馈字段；返回值表示解码结果。 */
static uint16_t M2006_ReadU16Be(const uint8_t *data)
{
    return ((uint16_t)data[0] << 8U) | data[1];
}

/* 功能：计算跨越编码器零点后的最短计数差；用途：展开多圈转子位置；返回值表示带方向的增量计数。 */
static int32_t M2006_EncoderDelta(uint16_t encoder,
                                   uint16_t previous_encoder)
{
    int32_t delta;

    delta = (int32_t)encoder - (int32_t)previous_encoder;
    if (delta > ((int32_t)M2006_ENCODER_COUNTS / 2))
    {
        delta -= (int32_t)M2006_ENCODER_COUNTS;
    }
    else if (delta < -((int32_t)M2006_ENCODER_COUNTS / 2))
    {
        delta += (int32_t)M2006_ENCODER_COUNTS;
    }
    return delta;
}

/* 功能：清空 M2006 单个 PID 的积分和历史误差；用途：模式切换或停机后重新起算；无返回值表示运行状态已复位。 */
static void M2006_ResetPid(m2006_pid_t *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->previous_valid = false;
}

/* 功能：执行一次带积分和输出限幅的 PID 计算；用途：实现位置环或基础速度环；返回值表示限幅后的控制量。 */
static float M2006_PidCalc(m2006_pid_t *pid,
                           float error,
                           float dt_s,
                           float output_limit)
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
        candidate_integral = M2006_Clamp(
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
    output = M2006_Clamp(unsaturated_output, -output_limit, output_limit);
    if ((integration_weight > 0.0f) &&
        (unsaturated_output != output) &&
        ((error * unsaturated_output) > 0.0f))
    {
        candidate_integral = previous_integral;
        unsaturated_output = pid->cfg.kp * error +
                             pid->cfg.ki * candidate_integral +
                             pid->cfg.kd * derivative;
        output = M2006_Clamp(unsaturated_output,
                             -output_limit,
                             output_limit);
    }
    pid->integral = candidate_integral;
    pid->previous_error = error;
    pid->previous_valid = true;
    return output;
}

/* 功能：按当前速度 PID 配置在线调参器；用途：建立自适应增益边界和学习参数；返回 true 表示初始化成功。 */
static bool M2006_ConfigureOnlinePid(m2006_context_t *context,
                                     bool enabled)
{
    motor_online_pid_cfg_t cfg;
    const m2006_pid_cfg_t *base;

    base = &context->speed_pid_base;
    (void)memset(&cfg, 0, sizeof(cfg));
    cfg.strategy = MOTOR_ONLINE_STRATEGY_ADAPTIVE;
    cfg.base_gains.kp = base->kp;
    cfg.base_gains.ki = base->ki;
    cfg.base_gains.kd = base->kd;
    cfg.minimum_gains.kp = base->kp * 0.65f;
    cfg.minimum_gains.ki = base->ki * 0.50f;
    cfg.minimum_gains.kd = base->kd * 0.50f;
    cfg.maximum_gains.kp = base->kp * 1.45f;
    cfg.maximum_gains.ki = base->ki * 1.80f;
    cfg.maximum_gains.kd = base->kd * 1.50f;
    cfg.learning_rate.kp = base->kp * 0.0008f;
    cfg.learning_rate.ki = base->ki * 0.0004f;
    cfg.learning_rate.kd = 0.0f;
    cfg.adaptation_deadband = 0.052360f;
    cfg.normalization_epsilon = 0.005236f;
    cfg.adaptation_leak_per_s = 0.08f;
    cfg.small_error_threshold = 0.209440f;
    cfg.large_error_threshold = 3.141593f;
    cfg.error_rate_threshold = 31.415927f;
    cfg.boost_factor = 1.20f;
    cfg.damping_factor = 0.70f;
    cfg.smoothing = 0.12f;
    return MotorOnlinePid_Init(&context->speed_online_pid, &cfg, enabled);
}

/* 功能：使用在线增益执行 M2006 速度环计算；用途：把目标转速转换为电流命令；返回值表示限幅后的电流安培值。 */
static float M2006_SpeedPidCalc(m2006_context_t *context,
                                float error,
                                float dt_s,
                                float output_limit)
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
    return M2006_PidCalc(&context->speed_pid,
                         error,
                         dt_s,
                         output_limit);
}

/* 功能：将安培值换算并限制为 M2006 原始电流字段；用途：填充 DJI 分组帧；返回值表示协议电流值。 */
static int16_t M2006_CurrentToRaw(float current_a)
{
    float raw;

    current_a = M2006_Clamp(current_a,
                            -M2006_CURRENT_MAX_A,
                            M2006_CURRENT_MAX_A);
    raw = current_a * (float)M2006_CURRENT_RAW_MAX / M2006_CURRENT_MAX_A;
    if (raw >= 0.0f)
    {
        return (int16_t)(raw + 0.5f);
    }
    return (int16_t)(raw - 0.5f);
}

/* 功能：复位 M2006 双环 PID、轨迹、斜坡和在线调参状态；用途：停机或控制模式改变时清除历史；无返回值表示状态已归零。 */
static void M2006_ResetControl(m2006_context_t *context)
{
    M2006_ResetPid(&context->speed_pid);
    MotorOnlinePid_Reset(&context->speed_online_pid, true);
    context->speed_pid.cfg = context->speed_pid_base;
    M2006_ResetPid(&context->position_pid);
    context->speed_reference_valid = false;
    context->trajectory_pending = false;
    context->motion.trajectory_active = false;
    context->motion.trajectory_progress = 1.0f;
    context->motion.reference_acceleration_rad_s2 = 0.0f;
}

/* 功能：以当前反馈为起点建立新的位置轨迹；用途：让位置目标按限速和加速度平滑变化；无返回值表示轨迹状态已初始化。 */
static void M2006_BeginPositionTrajectory(m2006_context_t *context,
                                           float position_rad)
{
    float distance_rad;
    float speed_duration_s;
    float acceleration_duration_s;

    distance_rad = context->target - position_rad;
    context->trajectory_start_rad = position_rad;
    context->trajectory_delta_rad = distance_rad;
    speed_duration_s = M2006_QUINTIC_MAX_SPEED * fabsf(distance_rad) /
                       context->position_vel_limit_rad_s;
    acceleration_duration_s = sqrtf(M2006_QUINTIC_MAX_ACCEL *
                                    fabsf(distance_rad) /
                                    context->acceleration_limit_rad_s2);
    context->trajectory_duration_s =
        (speed_duration_s > acceleration_duration_s) ?
        speed_duration_s : acceleration_duration_s;
    if (context->trajectory_duration_s < M2006_MIN_TRAJECTORY_S)
    {
        context->trajectory_duration_s = M2006_MIN_TRAJECTORY_S;
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
static void M2006_UpdatePositionTrajectory(m2006_context_t *context,
                                            float dt_s)
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
static void M2006_UpdateSpeedRamp(m2006_context_t *context,
                                  float actual_velocity_rad_s,
                                  float dt_s)
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
    step_rad_s = M2006_Clamp(delta_rad_s,
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
        context->motion.trajectory_progress = M2006_Clamp(
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

/* 功能：将 M2006 自动整定标记为失败并清理测试输出；用途：统一处理超时或非法工况；无返回值表示整定已终止。 */
static void M2006_FailAutoTune(m2006_context_t *context)
{
    context->auto_tune.state.status = M2006_AUTOTUNE_FAILED;
    context->mode = M2006_MODE_STOP;
    context->target = 0.0f;
    context->motion.current_command_a = 0.0f;
}

/* 功能：执行 M2006 速度环自动整定的一步状态机；用途：施加测试激励并估算 PID；返回值表示本周期测试电流。 */
static float M2006_AutoTuneStep(m2006_context_t *context,
                                float velocity_rad_s,
                                uint32_t tick_ms)
{
    m2006_autotune_t *tune;

    tune = &context->auto_tune;
    if (((tick_ms - tune->started_ms) > M2006_AUTOTUNE_TIMEOUT_MS) ||
        (fabsf(velocity_rad_s) > tune->safety_velocity_rad_s))
    {
        M2006_FailAutoTune(context);
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
    if (tune->completed_cycles >= M2006_AUTOTUNE_CYCLES)
    {
        float period_s;
        float amplitude_rad_s;
        float ultimate_gain;

        period_s = tune->period_sum_s / (float)tune->completed_cycles;
        amplitude_rad_s = tune->amplitude_sum_rad_s /
                          (float)tune->completed_cycles;
        ultimate_gain = 4.0f * tune->state.relay_current_a /
                        (0.5f * M2006_TWO_PI * amplitude_rad_s);
        tune->state.oscillation_amplitude_rad_s = amplitude_rad_s;
        tune->state.ultimate_period_s = period_s;
        tune->state.ultimate_gain = ultimate_gain;
        tune->state.result = context->speed_pid_base;
        tune->state.result.kp = ultimate_gain / 3.2f;
        tune->state.result.ki = tune->state.result.kp / (2.2f * period_s);
        tune->state.result.kd = 0.0f;
        context->speed_pid_base = tune->state.result;
        context->speed_pid.cfg = tune->state.result;
        (void)M2006_ConfigureOnlinePid(
            context, context->speed_online_pid.enabled != 0U);
        M2006_ResetPid(&context->speed_pid);
        tune->state.status = M2006_AUTOTUNE_COMPLETE;
        context->mode = M2006_MODE_STOP;
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

/* 功能：根据反馈新鲜度更新 M2006 超时统计；用途：累计连续丢帧、恢复和最大间隔；无返回值表示统计已刷新。 */
static void M2006_UpdateTimeoutStats(m2006_context_t *context,
                                     bool feedback_valid,
                                     const m2006_feedback_t *feedback,
                                     uint32_t tick_ms)
{
    bool command_timed_out;
    bool feedback_timed_out;

    command_timed_out = false;
    feedback_timed_out = false;
    if (context->mode != M2006_MODE_STOP)
    {
        command_timed_out =
            (tick_ms - context->command_updated_at_ms) >
            m2006_cfg.command_timeout_ms;
        if (feedback_valid)
        {
            feedback_timed_out =
                (tick_ms - feedback->updated_at_ms) >
                m2006_cfg.feedback_timeout_ms;
        }
        else
        {
            feedback_timed_out =
                (tick_ms - context->feedback_monitor_started_at_ms) >
                m2006_cfg.feedback_timeout_ms;
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

/* 功能：初始化全部 M2006 上下文和默认控制参数；用途：建立各 CAN 总线的反馈与双环控制状态；返回 true 表示配置被接受。 */
bool M2006_Init(const m2006_cfg_t *cfg)
{
    uint32_t bus_index;
    uint32_t motor_index;

    if ((cfg == NULL) || !M2006_IsFinite(cfg->current_limit_a) ||
        !M2006_IsFinite(cfg->position_vel_limit_rad_s) ||
        !M2006_IsFinite(cfg->acceleration_limit_rad_s2) ||
        (cfg->current_limit_a <= 0.0f) ||
        (cfg->current_limit_a > M2006_CURRENT_MAX_A) ||
        (cfg->position_vel_limit_rad_s <= 0.0f) ||
        (cfg->acceleration_limit_rad_s2 <= 0.0f) ||
        (cfg->feedback_timeout_ms == 0U) ||
        (cfg->command_timeout_ms == 0U) ||
        !M2006_IsValidPid(&cfg->speed_pid) ||
        !M2006_IsValidPid(&cfg->position_pid))
    {
        return false;
    }

    m2006_cfg = *cfg;
    (void)memset(m2006_context, 0, sizeof(m2006_context));
    for (bus_index = 0U; bus_index < M2006_CAN_BUS_COUNT; bus_index++)
    {
        for (motor_index = 0U; motor_index < M2006_MOTOR_COUNT;
             motor_index++)
        {
            m2006_context_t *context;

            context = &m2006_context[bus_index][motor_index];
            context->mode = M2006_MODE_STOP;
            context->current_limit_a = cfg->current_limit_a;
            context->position_vel_limit_rad_s =
                cfg->position_vel_limit_rad_s;
            context->acceleration_limit_rad_s2 =
                cfg->acceleration_limit_rad_s2;
            context->motion.trajectory_progress = 1.0f;
            context->auto_tune.state.status = M2006_AUTOTUNE_IDLE;
            context->speed_pid.cfg = cfg->speed_pid;
            context->speed_pid.full_integral_error = 0.209440f;
            context->speed_pid.integral_separation_error = 3.141593f;
            context->speed_pid_base = cfg->speed_pid;
            if (!M2006_ConfigureOnlinePid(context, false))
            {
                return false;
            }
            context->position_pid.cfg = cfg->position_pid;
            context->position_pid.full_integral_error = 0.01f;
            context->position_pid.integral_separation_error = 0.20f;
            context->feedback.can_bus = (uint8_t)(bus_index + 1U);
            context->feedback.motor_id = (uint8_t)(motor_index + 1U);
        }
    }
    return true;
}

/* 功能：设置指定 M2006 的停止、电流、速度或位置目标；用途：更新下一控制周期的命令；返回 true 表示模式和目标合法。 */
bool M2006_SetTarget(uint8_t can_bus,
                     uint8_t motor_id,
                     m2006_mode_t mode,
                     float target,
                     uint32_t tick_ms)
{
    m2006_context_t *context;
    bool mode_changed;
    bool target_changed;

    context = M2006_GetContext(can_bus, motor_id);
    if ((context == NULL) || !M2006_IsFinite(target) ||
        (mode > M2006_MODE_POSITION))
    {
        return false;
    }
    mode_changed = context->mode != mode;
    target_changed = mode_changed || (fabsf(context->target - target) > 0.000001f);
    if ((context->mode == M2006_MODE_STOP) &&
        (mode != M2006_MODE_STOP))
    {
        context->feedback_monitor_started_at_ms = tick_ms;
    }
    if (mode_changed)
    {
        M2006_ResetControl(context);
    }
    context->auto_tune.state.status = M2006_AUTOTUNE_IDLE;
    context->mode = mode;
    context->target = target;
    context->motion.final_position_rad =
        (mode == M2006_MODE_POSITION) ? target : 0.0f;
    context->motion.final_velocity_rad_s =
        (mode == M2006_MODE_VELOCITY) ? target : 0.0f;
    context->motion.reference_acceleration_rad_s2 = 0.0f;
    if (mode == M2006_MODE_POSITION)
    {
        if (target_changed)
        {
            context->trajectory_pending = true;
        }
        context->speed_reference_valid = false;
    }
    else if (mode == M2006_MODE_VELOCITY)
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

/* 功能：读取指定 M2006 的最新反馈；用途：获取编码器、速度、电流和温度；返回 true 表示已收到有效帧。 */
bool M2006_GetFeedback(uint8_t can_bus,
                       uint8_t motor_id,
                       m2006_feedback_t *feedback)
{
    const m2006_context_t *context;
    uint32_t before;
    uint32_t after;
    bool valid;

    context = M2006_GetContext(can_bus, motor_id);
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

/* 功能：读取带零点和多圈位置的 M2006 反馈快照；用途：供诊断与位置显示使用；返回 true 表示快照有效。 */
bool M2006_GetFeedbackSnapshot(uint8_t can_bus,
                               uint8_t motor_id,
                               m2006_feedback_t *feedback,
                               bool *zero_valid)
{
    const m2006_context_t *context;
    uint32_t before;
    uint32_t after;
    bool valid;

    context = M2006_GetContext(can_bus, motor_id);
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

/* 功能：读取指定 M2006 的通信超时统计；用途：诊断丢帧、恢复次数和帧间隔；返回 true 表示统计已写出。 */
bool M2006_GetTimeoutStats(uint8_t can_bus,
                           uint8_t motor_id,
                           m2006_timeout_stats_t *stats)
{
    const m2006_context_t *context;

    context = M2006_GetContext(can_bus, motor_id);
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

/* 功能：解析 M2006 DJI 反馈帧并展开位置；用途：更新闭环反馈、时间戳和通信统计；返回 true 表示帧地址有效。 */
bool M2006_OnFrame(uint8_t can_bus,
                    uint8_t motor_id,
                    const can_frame_t *frame,
                    uint32_t tick_ms)
{
    m2006_context_t *context;
    volatile m2006_feedback_t *feedback;
    uint16_t encoder;
    int16_t rotor_speed_rpm;
    int32_t delta;
    int64_t total_counts;
    int64_t relative_counts;
    uint32_t sequence;

    context = M2006_GetContext(can_bus, motor_id);
    if ((context == NULL) || (frame == NULL) || frame->extended ||
        (frame->dlc != 8U) || (frame->id != (0x200U + motor_id)))
    {
        return false;
    }

    encoder = M2006_ReadU16Be(frame->data);
    rotor_speed_rpm = (int16_t)M2006_ReadU16Be(&frame->data[2]);
    if (encoder >= M2006_ENCODER_COUNTS)
    {
        return false;
    }

    feedback = &context->feedback;
    sequence = context->feedback_sequence;
    context->feedback_sequence = sequence + 1U;

    if (!context->feedback_valid)
    {
        if ((rotor_speed_rpm < -M2006_ZERO_MAX_SPEED_RPM) ||
            (rotor_speed_rpm > M2006_ZERO_MAX_SPEED_RPM))
        {
            context->zero_stable_frames = 0U;
        }
        else if (context->zero_stable_frames == 0U)
        {
            context->zero_stable_frames = 1U;
        }
        else
        {
            delta = M2006_EncoderDelta(encoder,
                                        context->previous_encoder);
            if ((delta >= -M2006_ZERO_MAX_STEP_COUNTS) &&
                (delta <= M2006_ZERO_MAX_STEP_COUNTS))
            {
                if (context->zero_stable_frames <
                    M2006_ZERO_STABLE_FRAMES)
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
        delta = M2006_EncoderDelta(encoder,
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
        (int16_t)M2006_ReadU16Be(&frame->data[4]);
    feedback->total_encoder_counts = total_counts;
    feedback->zero_encoder_counts = context->zero_encoder_counts;
    feedback->output_pos_rad =
        (float)relative_counts * M2006_TWO_PI /
        ((float)M2006_ENCODER_COUNTS * M2006_REDUCTION_RATIO);
    feedback->output_vel_rad_s =
        (float)feedback->rotor_speed_rpm * M2006_TWO_PI /
        (60.0f * M2006_REDUCTION_RATIO);
    feedback->torque_current_a =
        (float)feedback->torque_current_raw * M2006_CURRENT_MAX_A /
        (float)M2006_CURRENT_RAW_MAX;
    feedback->updated_at_ms = tick_ms;
    feedback->rx_frames++;
    context->feedback_valid =
        context->zero_stable_frames >= M2006_ZERO_STABLE_FRAMES;
    context->feedback_sequence = sequence + 2U;
    return true;
}

/* 功能：根据当前模式和反馈计算 M2006 原始电流命令；用途：驱动双环控制、轨迹和自动整定；返回 true 表示成功写出电流。 */
bool M2006_CalcCurrentRaw(uint8_t can_bus,
                          uint8_t motor_id,
                          uint32_t tick_ms,
                          int16_t *current_raw)
{
    m2006_context_t *context;
    m2006_feedback_t feedback;
    float current_a;
    bool feedback_valid;

    context = M2006_GetContext(can_bus, motor_id);
    if ((context == NULL) || (current_raw == NULL))
    {
        return false;
    }

    current_a = 0.0f;
    feedback_valid = M2006_GetFeedback(can_bus, motor_id, &feedback);
    M2006_UpdateTimeoutStats(context, feedback_valid, &feedback, tick_ms);
    if (context->auto_tune.state.status == M2006_AUTOTUNE_RUNNING)
    {
        if (!feedback_valid ||
            ((tick_ms - feedback.updated_at_ms) >
             m2006_cfg.feedback_timeout_ms))
        {
            M2006_FailAutoTune(context);
            *current_raw = 0;
            return true;
        }
        current_a = M2006_AutoTuneStep(context,
                                       feedback.output_vel_rad_s,
                                       tick_ms);
        *current_raw = M2006_CurrentToRaw(current_a);
        return true;
    }
    if ((context->mode != M2006_MODE_STOP) &&
        (!feedback_valid || context->timeout_stats.command_timed_out ||
         context->timeout_stats.feedback_timed_out))
    {
        M2006_ResetControl(context);
        context->trajectory_pending =
            context->mode == M2006_MODE_POSITION;
        context->motion.current_command_a = 0.0f;
        *current_raw = 0;
        return true;
    }
    if (context->mode == M2006_MODE_STOP)
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
        M2006_ResetControl(context);
    }
    else
    {
        switch (context->mode)
        {
        case M2006_MODE_CURRENT:
            current_a = context->target;
            context->motion.reference_position_rad = feedback.output_pos_rad;
            context->motion.reference_velocity_rad_s =
                feedback.output_vel_rad_s;
            context->motion.reference_acceleration_rad_s2 = 0.0f;
            context->motion.position_error_rad = 0.0f;
            context->motion.velocity_error_rad_s = 0.0f;
            break;

        case M2006_MODE_VELOCITY:
            if (feedback_valid)
            {
                M2006_UpdateSpeedRamp(context,
                                      feedback.output_vel_rad_s,
                                      0.001f);
                context->motion.reference_position_rad =
                    feedback.output_pos_rad;
                context->motion.position_error_rad = 0.0f;
                context->motion.velocity_error_rad_s =
                    context->motion.reference_velocity_rad_s -
                    feedback.output_vel_rad_s;
                current_a = M2006_SpeedPidCalc(
                    context,
                    context->motion.velocity_error_rad_s,
                    0.001f,
                    context->current_limit_a);
            }
            break;

        case M2006_MODE_POSITION:
        {
            float target_vel_rad_s;

            if (feedback_valid)
            {
                if (context->trajectory_pending)
                {
                    M2006_BeginPositionTrajectory(
                        context, feedback.output_pos_rad);
                }
                M2006_UpdatePositionTrajectory(context, 0.001f);
                context->motion.position_error_rad =
                    context->motion.reference_position_rad -
                    feedback.output_pos_rad;
                target_vel_rad_s = M2006_PidCalc(
                    &context->position_pid,
                    context->motion.position_error_rad,
                    0.001f,
                    context->position_vel_limit_rad_s);
                target_vel_rad_s = M2006_Clamp(
                    context->motion.reference_velocity_rad_s +
                    target_vel_rad_s,
                    -context->position_vel_limit_rad_s,
                    context->position_vel_limit_rad_s);
                context->motion.velocity_error_rad_s =
                    target_vel_rad_s - feedback.output_vel_rad_s;
                current_a = M2006_SpeedPidCalc(
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

    current_a = M2006_Clamp(current_a,
                            -context->current_limit_a,
                            context->current_limit_a);
    context->motion.current_command_a = current_a;
    *current_raw = M2006_CurrentToRaw(current_a);
    return true;
}

/* 功能：设置指定 M2006 的速度环 PID；用途：调整速度响应并重新配置在线调参；返回 true 表示参数有效。 */
bool M2006_SetSpeedPid(uint8_t can_bus,
                       uint8_t motor_id,
                       const m2006_pid_cfg_t *cfg)
{
    m2006_context_t *context;
    bool enabled;

    context = M2006_GetContext(can_bus, motor_id);
    if ((context == NULL) || !M2006_IsValidPid(cfg))
    {
        return false;
    }
    enabled = context->speed_online_pid.enabled != 0U;
    context->speed_pid_base = *cfg;
    context->speed_pid.cfg = *cfg;
    if (!M2006_ConfigureOnlinePid(context, enabled))
    {
        return false;
    }
    M2006_ResetPid(&context->speed_pid);
    return true;
}

/* 功能：设置指定 M2006 的位置环 PID；用途：调整位置到速度的外环响应；返回 true 表示参数有效。 */
bool M2006_SetPositionPid(uint8_t can_bus,
                          uint8_t motor_id,
                          const m2006_pid_cfg_t *cfg)
{
    m2006_context_t *context;

    context = M2006_GetContext(can_bus, motor_id);
    if ((context == NULL) || !M2006_IsValidPid(cfg))
    {
        return false;
    }
    context->position_pid.cfg = *cfg;
    M2006_ResetPid(&context->position_pid);
    return true;
}

/* 功能：设置指定 M2006 的软件电流限制；用途：约束所有闭环和直接控制输出；返回 true 表示限制合法。 */
bool M2006_SetCurrentLimit(uint8_t can_bus,
                           uint8_t motor_id,
                           float current_limit_a)
{
    m2006_context_t *context;

    context = M2006_GetContext(can_bus, motor_id);
    if ((context == NULL) || !M2006_IsFinite(current_limit_a) ||
        (current_limit_a <= 0.0f) ||
        (current_limit_a > M2006_CURRENT_MAX_A))
    {
        return false;
    }
    context->current_limit_a = current_limit_a;
    return true;
}

/* 功能：设置 M2006 位置模式的最大速度；用途：限制位置轨迹运动速度；返回 true 表示限制合法。 */
bool M2006_SetPositionVelocityLimit(uint8_t can_bus,
                                    uint8_t motor_id,
                                    float velocity_limit_rad_s)
{
    m2006_context_t *context;

    context = M2006_GetContext(can_bus, motor_id);
    if ((context == NULL) || !M2006_IsFinite(velocity_limit_rad_s) ||
        (velocity_limit_rad_s <= 0.0f))
    {
        return false;
    }
    context->position_vel_limit_rad_s = velocity_limit_rad_s;
    return true;
}

/* 功能：设置 M2006 速度变化的加速度限制；用途：平滑速度和位置轨迹；返回 true 表示限制合法。 */
bool M2006_SetAccelerationLimit(uint8_t can_bus,
                                uint8_t motor_id,
                                float acceleration_limit_rad_s2)
{
    m2006_context_t *context;

    context = M2006_GetContext(can_bus, motor_id);
    if ((context == NULL) || !M2006_IsFinite(acceleration_limit_rad_s2) ||
        (acceleration_limit_rad_s2 <= 0.0f))
    {
        return false;
    }
    context->acceleration_limit_rad_s2 = acceleration_limit_rad_s2;
    return true;
}

/* 功能：启用或关闭指定 M2006 的速度环在线调参；用途：切换固定与自适应 PID；返回 true 表示设置成功。 */
bool M2006_SetOnlinePidEnabled(uint8_t can_bus,
                               uint8_t motor_id,
                               bool enabled)
{
    m2006_context_t *context;

    context = M2006_GetContext(can_bus, motor_id);
    if (context == NULL)
    {
        return false;
    }
    MotorOnlinePid_SetEnabled(&context->speed_online_pid, enabled);
    context->speed_pid.cfg = context->speed_pid_base;
    M2006_ResetPid(&context->speed_pid);
    return true;
}

/* 功能：把当前 M2006 多圈位置记录为软件零点；用途：建立输出轴相对位置基准；返回 true 表示已有反馈且置零成功。 */
bool M2006_ZeroPosition(uint8_t can_bus, uint8_t motor_id)
{
    m2006_context_t *context;

    context = M2006_GetContext(can_bus, motor_id);
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
    M2006_ResetPid(&context->position_pid);
    return true;
}

/* 功能：读取指定 M2006 的在线 PID 调参状态；用途：观察当前增益、误差和活动规则；返回 true 表示状态已写出。 */
bool M2006_GetOnlinePidState(uint8_t can_bus,
                             uint8_t motor_id,
                             m2006_online_pid_state_t *state)
{
    const m2006_context_t *context;

    context = M2006_GetContext(can_bus, motor_id);
    if ((context == NULL) || (state == NULL))
    {
        return false;
    }
    state->enabled = context->speed_online_pid.enabled != 0U;
    state->strategy = state->enabled ?
                      (uint8_t)MOTOR_ONLINE_STRATEGY_ADAPTIVE : 0U;
    state->active_rule =
        (uint8_t)context->speed_online_pid.active_rule;
    state->applied_kp = context->speed_online_pid.applied_gains.kp;
    state->applied_ki = context->speed_online_pid.applied_gains.ki;
    state->applied_kd = context->speed_online_pid.applied_gains.kd;
    return true;
}

/* 功能：读取 M2006 内部斜坡和位置轨迹状态；用途：诊断当前平滑给定及轨迹是否活动；返回 true 表示状态有效。 */
bool M2006_GetMotionState(uint8_t can_bus,
                          uint8_t motor_id,
                          m2006_motion_state_t *state)
{
    const m2006_context_t *context;

    context = M2006_GetContext(can_bus, motor_id);
    if ((context == NULL) || (state == NULL))
    {
        return false;
    }
    *state = context->motion;
    return true;
}

/* 功能：读取指定 M2006 的速度环 PID 配置；用途：显示或保存当前控制参数；返回 true 表示配置已写出。 */
bool M2006_GetSpeedPid(uint8_t can_bus,
                       uint8_t motor_id,
                       m2006_pid_cfg_t *cfg)
{
    const m2006_context_t *context;

    context = M2006_GetContext(can_bus, motor_id);
    if ((context == NULL) || (cfg == NULL))
    {
        return false;
    }
    *cfg = context->speed_pid_base;
    return true;
}

/* 功能：读取指定 M2006 的位置环 PID 配置；用途：显示或保存当前控制参数；返回 true 表示配置已写出。 */
bool M2006_GetPositionPid(uint8_t can_bus,
                          uint8_t motor_id,
                          m2006_pid_cfg_t *cfg)
{
    const m2006_context_t *context;

    context = M2006_GetContext(can_bus, motor_id);
    if ((context == NULL) || (cfg == NULL))
    {
        return false;
    }
    *cfg = context->position_pid.cfg;
    return true;
}

/* 功能：启动指定 M2006 的速度环自动整定；用途：通过继电反馈测试估算控制增益；返回 true 表示整定已进入运行态。 */
bool M2006_StartSpeedAutoTune(uint8_t can_bus,
                              uint8_t motor_id,
                              float relay_current_a,
                              float hysteresis_rad_s,
                              float safety_velocity_rad_s,
                              uint32_t tick_ms)
{
    m2006_context_t *context;
    m2006_feedback_t feedback;

    context = M2006_GetContext(can_bus, motor_id);
    if ((context == NULL) || !M2006_IsFinite(relay_current_a) ||
        !M2006_IsFinite(hysteresis_rad_s) ||
        !M2006_IsFinite(safety_velocity_rad_s) ||
        (relay_current_a <= 0.0f) || (hysteresis_rad_s <= 0.0f) ||
        (safety_velocity_rad_s <= hysteresis_rad_s) ||
        !M2006_GetFeedback(can_bus, motor_id, &feedback) ||
        ((tick_ms - feedback.updated_at_ms) > m2006_cfg.feedback_timeout_ms) ||
        (relay_current_a > context->current_limit_a) ||
        (fabsf(feedback.output_vel_rad_s) >= safety_velocity_rad_s))
    {
        return false;
    }
    (void)memset(&context->auto_tune, 0, sizeof(context->auto_tune));
    context->auto_tune.state.status = M2006_AUTOTUNE_RUNNING;
    context->auto_tune.state.relay_current_a = relay_current_a;
    context->auto_tune.state.hysteresis_rad_s = hysteresis_rad_s;
    context->auto_tune.state.result = context->speed_pid_base;
    context->auto_tune.safety_velocity_rad_s = safety_velocity_rad_s;
    context->auto_tune.phase_max_rad_s = feedback.output_vel_rad_s;
    context->auto_tune.phase_min_rad_s = feedback.output_vel_rad_s;
    context->auto_tune.started_ms = tick_ms;
    context->auto_tune.relay_positive = true;
    context->mode = M2006_MODE_STOP;
    context->target = 0.0f;
    M2006_ResetControl(context);
    return true;
}

/* 功能：取消正在进行的 M2006 自动整定；用途：响应用户停止或安全条件变化；返回 true 表示地址有效并已取消。 */
bool M2006_CancelAutoTune(uint8_t can_bus, uint8_t motor_id)
{
    m2006_context_t *context;

    context = M2006_GetContext(can_bus, motor_id);
    if (context == NULL)
    {
        return false;
    }
    context->auto_tune.state.status = M2006_AUTOTUNE_IDLE;
    context->mode = M2006_MODE_STOP;
    context->target = 0.0f;
    context->motion.current_command_a = 0.0f;
    return true;
}

/* 功能：读取指定 M2006 的自动整定状态和结果；用途：向调试界面报告进度、参数或失败原因；返回 true 表示状态已写出。 */
bool M2006_GetAutoTuneState(uint8_t can_bus,
                            uint8_t motor_id,
                            m2006_autotune_state_t *state)
{
    const m2006_context_t *context;

    context = M2006_GetContext(can_bus, motor_id);
    if ((context == NULL) || (state == NULL))
    {
        return false;
    }
    *state = context->auto_tune.state;
    return true;
}
