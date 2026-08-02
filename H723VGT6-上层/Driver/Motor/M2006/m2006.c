#include "m2006.h"

#include <float.h>
#include <string.h>

#include "motor_online_tune.h"

#define M2006_ENCODER_COUNTS   8192U
#define M2006_REDUCTION_RATIO  36.0f
#define M2006_CURRENT_RAW_MAX  10000
#define M2006_CURRENT_MAX_A    10.0f
#define M2006_TWO_PI           6.28318530718f

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
    volatile uint32_t feedback_sequence;
    volatile bool feedback_valid;
    volatile m2006_feedback_t feedback;
    uint16_t previous_encoder;
    int64_t zero_encoder_counts;
    m2006_mode_t mode;
    float target;
    uint32_t command_updated_at_ms;
    uint32_t feedback_monitor_started_at_ms;
    volatile m2006_timeout_stats_t timeout_stats;
    m2006_pid_t speed_pid;
    m2006_pid_cfg_t speed_pid_base;
    motor_online_pid_t speed_online_pid;
    m2006_pid_t position_pid;
} m2006_context_t;

static m2006_cfg_t m2006_cfg;
static m2006_context_t
    m2006_context[M2006_CAN_BUS_COUNT][M2006_MOTOR_COUNT];

static bool M2006_IsFinite(float value)
{
    return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX);
}

static bool M2006_IsValidAddress(uint8_t can_bus, uint8_t motor_id)
{
    return (can_bus >= 1U) && (can_bus <= M2006_CAN_BUS_COUNT) &&
           (motor_id >= 1U) && (motor_id <= M2006_MOTOR_COUNT);
}

static bool M2006_IsValidPid(const m2006_pid_cfg_t *cfg)
{
    return (cfg != NULL) && M2006_IsFinite(cfg->kp) &&
           M2006_IsFinite(cfg->ki) && M2006_IsFinite(cfg->kd) &&
           M2006_IsFinite(cfg->integral_limit) &&
           M2006_IsFinite(cfg->output_limit) && (cfg->kp >= 0.0f) &&
           (cfg->ki >= 0.0f) && (cfg->kd >= 0.0f) &&
           (cfg->integral_limit >= 0.0f) && (cfg->output_limit > 0.0f);
}

static m2006_context_t *M2006_GetContext(uint8_t can_bus,
                                         uint8_t motor_id)
{
    if (!M2006_IsValidAddress(can_bus, motor_id))
    {
        return NULL;
    }
    return &m2006_context[can_bus - 1U][motor_id - 1U];
}

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

static uint16_t M2006_ReadU16Be(const uint8_t *data)
{
    return ((uint16_t)data[0] << 8U) | data[1];
}

static void M2006_ResetPid(m2006_pid_t *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->previous_valid = false;
}

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
    cfg.normalization_epsilon = 0.05f;
    cfg.adaptation_leak_per_s = 0.08f;
    cfg.small_error_threshold = 0.209440f;
    cfg.large_error_threshold = 3.141593f;
    cfg.error_rate_threshold = 31.415927f;
    cfg.boost_factor = 1.20f;
    cfg.damping_factor = 0.70f;
    cfg.smoothing = 0.12f;
    return MotorOnlinePid_Init(&context->speed_online_pid, &cfg, enabled);
}

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

static int16_t M2006_CurrentToRaw(float current_a)
{
    float raw;

    current_a = M2006_Clamp(current_a,
                            -m2006_cfg.current_limit_a,
                            m2006_cfg.current_limit_a);
    raw = current_a * (float)M2006_CURRENT_RAW_MAX / M2006_CURRENT_MAX_A;
    if (raw >= 0.0f)
    {
        return (int16_t)(raw + 0.5f);
    }
    return (int16_t)(raw - 0.5f);
}

static void M2006_ResetControl(m2006_context_t *context)
{
    M2006_ResetPid(&context->speed_pid);
    MotorOnlinePid_Reset(&context->speed_online_pid, true);
    context->speed_pid.cfg = context->speed_pid_base;
    M2006_ResetPid(&context->position_pid);
}

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

bool M2006_Init(const m2006_cfg_t *cfg)
{
    uint32_t bus_index;
    uint32_t motor_index;

    if ((cfg == NULL) || !M2006_IsFinite(cfg->current_limit_a) ||
        !M2006_IsFinite(cfg->position_vel_limit_rad_s) ||
        (cfg->current_limit_a <= 0.0f) ||
        (cfg->current_limit_a > M2006_CURRENT_MAX_A) ||
        (cfg->position_vel_limit_rad_s <= 0.0f) ||
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
            context->speed_pid.cfg = cfg->speed_pid;
            context->speed_pid.full_integral_error = 0.209440f;
            context->speed_pid.integral_separation_error = 3.141593f;
            context->speed_pid_base = cfg->speed_pid;
            if (!M2006_ConfigureOnlinePid(context, true))
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

bool M2006_SetTarget(uint8_t can_bus,
                     uint8_t motor_id,
                     m2006_mode_t mode,
                     float target,
                     uint32_t tick_ms)
{
    m2006_context_t *context;

    context = M2006_GetContext(can_bus, motor_id);
    if ((context == NULL) || !M2006_IsFinite(target) ||
        (mode > M2006_MODE_POSITION))
    {
        return false;
    }

    if ((context->mode == M2006_MODE_STOP) &&
        (mode != M2006_MODE_STOP))
    {
        context->feedback_monitor_started_at_ms = tick_ms;
    }
    if (context->mode != mode)
    {
        M2006_ResetControl(context);
    }
    context->mode = mode;
    context->target = target;
    context->command_updated_at_ms = tick_ms;
    return true;
}

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

bool M2006_OnFrame(uint8_t can_bus,
                    uint8_t motor_id,
                    const can_frame_t *frame,
                    uint32_t tick_ms)
{
    m2006_context_t *context;
    volatile m2006_feedback_t *feedback;
    uint16_t encoder;
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
    if (encoder >= M2006_ENCODER_COUNTS)
    {
        return false;
    }

    feedback = &context->feedback;
    sequence = context->feedback_sequence;
    context->feedback_sequence = sequence + 1U;

    if (!context->feedback_valid)
    {
        total_counts = encoder;
        context->zero_encoder_counts = total_counts;
    }
    else
    {
        delta = (int32_t)encoder - (int32_t)context->previous_encoder;
        if (delta > ((int32_t)M2006_ENCODER_COUNTS / 2))
        {
            delta -= (int32_t)M2006_ENCODER_COUNTS;
        }
        else if (delta < -((int32_t)M2006_ENCODER_COUNTS / 2))
        {
            delta += (int32_t)M2006_ENCODER_COUNTS;
        }
        total_counts = feedback->total_encoder_counts + delta;
    }
    context->previous_encoder = encoder;
    relative_counts = total_counts - context->zero_encoder_counts;

    feedback->can_bus = can_bus;
    feedback->motor_id = motor_id;
    feedback->rotor_encoder = encoder;
    feedback->rotor_speed_rpm =
        (int16_t)M2006_ReadU16Be(&frame->data[2]);
    feedback->torque_current_raw =
        (int16_t)M2006_ReadU16Be(&frame->data[4]);
    feedback->total_encoder_counts = total_counts;
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
    context->feedback_valid = true;
    context->feedback_sequence = sequence + 2U;
    return true;
}

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
    if (context->mode != M2006_MODE_STOP)
    {
        switch (context->mode)
        {
        case M2006_MODE_CURRENT:
            current_a = context->target;
            break;

        case M2006_MODE_VELOCITY:
            if (feedback_valid)
            {
                current_a = M2006_SpeedPidCalc(
                    context,
                    context->target - feedback.output_vel_rad_s,
                    0.001f,
                    m2006_cfg.current_limit_a);
            }
            break;

        case M2006_MODE_POSITION:
        {
            float target_vel_rad_s;

            if (feedback_valid)
            {
                target_vel_rad_s = M2006_PidCalc(
                    &context->position_pid,
                    context->target - feedback.output_pos_rad,
                    0.001f,
                    m2006_cfg.position_vel_limit_rad_s);
                current_a = M2006_SpeedPidCalc(
                    context,
                    target_vel_rad_s - feedback.output_vel_rad_s,
                    0.001f,
                    m2006_cfg.current_limit_a);
            }
            break;
        }

        default:
            break;
        }
    }

    *current_raw = M2006_CurrentToRaw(current_a);
    return true;
}

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
