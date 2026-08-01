#include "m2006.h"

#include <float.h>
#include <string.h>

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
    float output;

    derivative = 0.0f;
    if (pid->previous_valid)
    {
        derivative = (error - pid->previous_error) / dt_s;
    }

    pid->integral += error * dt_s;
    if (pid->cfg.integral_limit > 0.0f)
    {
        pid->integral = M2006_Clamp(pid->integral,
                                    -pid->cfg.integral_limit,
                                    pid->cfg.integral_limit);
    }
    else
    {
        pid->integral = 0.0f;
    }

    if (output_limit > pid->cfg.output_limit)
    {
        output_limit = pid->cfg.output_limit;
    }
    output = pid->cfg.kp * error + pid->cfg.ki * pid->integral +
             pid->cfg.kd * derivative;
    output = M2006_Clamp(output, -output_limit, output_limit);
    pid->previous_error = error;
    pid->previous_valid = true;
    return output;
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
            context->position_pid.cfg = cfg->position_pid;
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
                current_a = M2006_PidCalc(
                    &context->speed_pid,
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
                current_a = M2006_PidCalc(
                    &context->speed_pid,
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
