#include "m3508.h"

#include <float.h>
#include <string.h>

#define M3508_ENCODER_COUNTS         8192U
#define M3508_REDUCTION_RATIO        (3591.0f / 187.0f)
#define M3508_CURRENT_RAW_MAX        16384
#define M3508_CURRENT_MAX_A          20.0f
#define M3508_TWO_PI                 6.28318530718f

typedef struct
{
    m3508_pid_cfg_t cfg;
    float integral;
    float previous_error;
    bool previous_valid;
} m3508_pid_t;

typedef struct
{
    volatile uint32_t feedback_sequence;
    volatile bool feedback_valid;
    volatile m3508_feedback_t feedback;
    uint16_t previous_encoder;
    int64_t zero_encoder_counts;
    m3508_mode_t mode;
    float target;
    uint32_t command_updated_at_ms;
    uint32_t feedback_monitor_started_at_ms;
    volatile m3508_timeout_stats_t timeout_stats;
    m3508_pid_t speed_pid;
    m3508_pid_t position_pid;
} m3508_context_t;

static m3508_cfg_t m3508_cfg;
static m3508_context_t m3508_context[M3508_MOTOR_COUNT];

static bool M3508_IsFinite(float value)
{
    return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX);
}

static bool M3508_IsValidId(uint8_t motor_id)
{
    return (motor_id >= 1U) && (motor_id <= M3508_MOTOR_COUNT);
}

static bool M3508_IsValidPid(const m3508_pid_cfg_t *cfg)
{
    return (cfg != NULL) && M3508_IsFinite(cfg->kp) &&
           M3508_IsFinite(cfg->ki) && M3508_IsFinite(cfg->kd) &&
           M3508_IsFinite(cfg->integral_limit) &&
           M3508_IsFinite(cfg->output_limit) && (cfg->kp >= 0.0f) &&
           (cfg->ki >= 0.0f) && (cfg->kd >= 0.0f) &&
           (cfg->integral_limit >= 0.0f) && (cfg->output_limit > 0.0f);
}

static float M3508_Clamp(float value, float min, float max)
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

static uint16_t M3508_ReadU16Be(const uint8_t *data)
{
    return ((uint16_t)data[0] << 8U) | data[1];
}

static void M3508_ResetPid(m3508_pid_t *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->previous_valid = false;
}

static float M3508_PidCalc(m3508_pid_t *pid,
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
        pid->integral = M3508_Clamp(pid->integral,
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
    output = M3508_Clamp(output, -output_limit, output_limit);
    pid->previous_error = error;
    pid->previous_valid = true;
    return output;
}

static int16_t M3508_CurrentToRaw(float current_a)
{
    float raw;

    current_a = M3508_Clamp(current_a,
                            -m3508_cfg.current_limit_a,
                            m3508_cfg.current_limit_a);
    raw = current_a * (float)M3508_CURRENT_RAW_MAX / M3508_CURRENT_MAX_A;
    if (raw >= 0.0f)
    {
        return (int16_t)(raw + 0.5f);
    }
    return (int16_t)(raw - 0.5f);
}

static void M3508_ResetControl(m3508_context_t *context)
{
    M3508_ResetPid(&context->speed_pid);
    M3508_ResetPid(&context->position_pid);
}

static void M3508_UpdateTimeoutStats(m3508_context_t *context,
                                     bool feedback_valid,
                                     const m3508_feedback_t *feedback,
                                     uint32_t tick_ms)
{
    bool command_timed_out;
    bool feedback_timed_out;

    command_timed_out = false;
    feedback_timed_out = false;
    if (context->mode != M3508_MODE_STOP)
    {
        command_timed_out =
            (tick_ms - context->command_updated_at_ms) >
            m3508_cfg.command_timeout_ms;
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

bool M3508_Init(const m3508_cfg_t *cfg)
{
    uint32_t index;

    if ((cfg == NULL) || !M3508_IsFinite(cfg->current_limit_a) ||
        !M3508_IsFinite(cfg->position_vel_limit_rad_s) ||
        (cfg->current_limit_a <= 0.0f) ||
        (cfg->current_limit_a > M3508_CURRENT_MAX_A) ||
        (cfg->position_vel_limit_rad_s <= 0.0f) ||
        (cfg->feedback_timeout_ms == 0U) ||
        (cfg->command_timeout_ms == 0U) ||
        !M3508_IsValidPid(&cfg->speed_pid) ||
        !M3508_IsValidPid(&cfg->position_pid))
    {
        return false;
    }

    m3508_cfg = *cfg;
    (void)memset(m3508_context, 0, sizeof(m3508_context));
    for (index = 0U; index < M3508_MOTOR_COUNT; index++)
    {
        m3508_context[index].mode = M3508_MODE_STOP;
        m3508_context[index].speed_pid.cfg = cfg->speed_pid;
        m3508_context[index].position_pid.cfg = cfg->position_pid;
        m3508_context[index].feedback.motor_id = (uint8_t)(index + 1U);
    }
    return true;
}

bool M3508_SetTarget(uint8_t motor_id,
                     m3508_mode_t mode,
                     float target,
                     uint32_t tick_ms)
{
    m3508_context_t *context;

    if (!M3508_IsValidId(motor_id) || !M3508_IsFinite(target) ||
        (mode < M3508_MODE_STOP) || (mode > M3508_MODE_POSITION))
    {
        return false;
    }

    context = &m3508_context[motor_id - 1U];
    if ((context->mode == M3508_MODE_STOP) &&
        (mode != M3508_MODE_STOP))
    {
        context->feedback_monitor_started_at_ms = tick_ms;
    }
    if (context->mode != mode)
    {
        M3508_ResetControl(context);
    }
    context->mode = mode;
    context->target = target;
    context->command_updated_at_ms = tick_ms;
    return true;
}

bool M3508_GetFeedback(uint8_t motor_id, m3508_feedback_t *feedback)
{
    const m3508_context_t *context;
    uint32_t before;
    uint32_t after;
    bool valid;

    if (!M3508_IsValidId(motor_id) || (feedback == NULL))
    {
        return false;
    }

    context = &m3508_context[motor_id - 1U];
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

bool M3508_GetTimeoutStats(uint8_t motor_id,
                           m3508_timeout_stats_t *stats)
{
    const m3508_context_t *context;

    if (!M3508_IsValidId(motor_id) || (stats == NULL))
    {
        return false;
    }

    context = &m3508_context[motor_id - 1U];
    stats->command_timed_out = context->timeout_stats.command_timed_out;
    stats->feedback_timed_out = context->timeout_stats.feedback_timed_out;
    stats->command_timeout_count =
        context->timeout_stats.command_timeout_count;
    stats->feedback_timeout_count =
        context->timeout_stats.feedback_timeout_count;
    return true;
}

bool M3508_OnFrame(uint8_t motor_id,
                    const can_frame_t *frame,
                    uint32_t tick_ms)
{
    m3508_context_t *context;
    volatile m3508_feedback_t *feedback;
    uint16_t encoder;
    int32_t delta;
    int64_t total_counts;
    int64_t relative_counts;
    uint32_t sequence;

    if (!M3508_IsValidId(motor_id) || (frame == NULL) ||
        frame->extended || (frame->dlc != 8U) ||
        (frame->id != (0x200U + motor_id)))
    {
        return false;
    }

    encoder = M3508_ReadU16Be(frame->data);
    if (encoder >= M3508_ENCODER_COUNTS)
    {
        return false;
    }

    context = &m3508_context[motor_id - 1U];
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
        if (delta > ((int32_t)M3508_ENCODER_COUNTS / 2))
        {
            delta -= (int32_t)M3508_ENCODER_COUNTS;
        }
        else if (delta < -((int32_t)M3508_ENCODER_COUNTS / 2))
        {
            delta += (int32_t)M3508_ENCODER_COUNTS;
        }
        total_counts = feedback->total_encoder_counts + delta;
    }
    context->previous_encoder = encoder;
    relative_counts = total_counts - context->zero_encoder_counts;

    feedback->motor_id = motor_id;
    feedback->rotor_encoder = encoder;
    feedback->rotor_speed_rpm = (int16_t)M3508_ReadU16Be(&frame->data[2]);
    feedback->torque_current_raw =
        (int16_t)M3508_ReadU16Be(&frame->data[4]);
    feedback->temperature_c = frame->data[6];
    feedback->total_encoder_counts = total_counts;
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
    context->feedback_valid = true;
    context->feedback_sequence = sequence + 2U;
    return true;
}

bool M3508_CalcCurrentRaw(uint8_t motor_id,
                          uint32_t tick_ms,
                          int16_t *current_raw)
{
    m3508_context_t *context;
    m3508_feedback_t feedback;
    float current_a;
    bool feedback_valid;

    if (!M3508_IsValidId(motor_id) || (current_raw == NULL))
    {
        return false;
    }

    context = &m3508_context[motor_id - 1U];
    current_a = 0.0f;
    feedback_valid = M3508_GetFeedback(motor_id, &feedback);
    M3508_UpdateTimeoutStats(context, feedback_valid, &feedback, tick_ms);
    if (context->mode != M3508_MODE_STOP)
    {
        switch (context->mode)
        {
        case M3508_MODE_CURRENT:
            current_a = context->target;
            break;

        case M3508_MODE_VELOCITY:
            if (feedback_valid)
            {
                current_a = M3508_PidCalc(
                    &context->speed_pid,
                    context->target - feedback.output_vel_rad_s,
                    0.001f,
                    m3508_cfg.current_limit_a);
            }
            break;

        case M3508_MODE_POSITION:
        {
            float target_vel_rad_s;

            if (feedback_valid)
            {
                target_vel_rad_s = M3508_PidCalc(
                    &context->position_pid,
                    context->target - feedback.output_pos_rad,
                    0.001f,
                    m3508_cfg.position_vel_limit_rad_s);
                current_a = M3508_PidCalc(
                    &context->speed_pid,
                    target_vel_rad_s - feedback.output_vel_rad_s,
                    0.001f,
                    m3508_cfg.current_limit_a);
            }
            break;
        }

        default:
            break;
        }
    }

    *current_raw = M3508_CurrentToRaw(current_a);
    return true;
}
