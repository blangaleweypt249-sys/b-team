#include "j4310.h"

#include <float.h>
#include <string.h>

#define J4310_CAN_STD_ID_MAX  0x7FFU
#define J4310_FEEDBACK_ID_MAX 0x0FU
#define J4310_KP_MAX          500.0f
#define J4310_KD_MAX          5.0f
#define J4310_CMD_ENABLE      0xFCU
#define J4310_CMD_DISABLE     0xFDU

typedef struct
{
    bool used;
    uint8_t motor_id;
    uint16_t master_id;
    uint8_t feedback_id;
    j4310_limits_t limits;
    volatile uint32_t sequence;
    volatile j4310_feedback_t feedback;
} j4310_context_t;

static j4310_context_t j4310_context[J4310_MAX_MOTOR_COUNT];

static bool J4310_IsFinite(float value)
{
    return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX);
}

static float J4310_Clamp(float value, float min, float max)
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

static uint16_t J4310_FloatToUint(float value,
                                  float min,
                                  float max,
                                  uint8_t bits)
{
    uint32_t scale;
    float normalized;

    scale = (1UL << bits) - 1UL;
    value = J4310_Clamp(value, min, max);
    normalized = (value - min) * (float)scale / (max - min);
    return (uint16_t)(normalized + 0.5f);
}

static float J4310_UintToFloat(uint16_t value,
                               float min,
                               float max,
                               uint8_t bits)
{
    uint32_t scale;

    scale = (1UL << bits) - 1UL;
    return ((float)value * (max - min) / (float)scale) + min;
}

static j4310_context_t *J4310_Find(uint8_t motor_id)
{
    uint32_t index;

    for (index = 0U; index < J4310_MAX_MOTOR_COUNT; index++)
    {
        if (j4310_context[index].used &&
            (j4310_context[index].motor_id == motor_id))
        {
            return &j4310_context[index];
        }
    }
    return NULL;
}

static bool J4310_BuildSpecial(uint8_t motor_id,
                               uint8_t command,
                               can_frame_t *frame)
{
    j4310_context_t *context;

    context = J4310_Find(motor_id);
    if ((context == NULL) || (frame == NULL))
    {
        return false;
    }

    (void)memset(frame, 0, sizeof(*frame));
    frame->id = context->motor_id;
    frame->dlc = 8U;
    (void)memset(frame->data, 0xFF, 7U);
    frame->data[7] = command;
    return true;
}

void J4310_Init(void)
{
    (void)memset(j4310_context, 0, sizeof(j4310_context));
}

bool J4310_AddMotor(uint8_t motor_id,
                    uint16_t master_id,
                    uint8_t feedback_id,
                    const j4310_limits_t *limits)
{
    uint32_t index;

    if ((motor_id == 0U) || (motor_id > J4310_FEEDBACK_ID_MAX) ||
        (master_id > J4310_CAN_STD_ID_MAX) ||
        (feedback_id > J4310_FEEDBACK_ID_MAX) ||
        (limits == NULL) ||
        !J4310_IsFinite(limits->position_max_rad) ||
        !J4310_IsFinite(limits->velocity_max_rad_s) ||
        !J4310_IsFinite(limits->torque_max_nm) ||
        (limits->position_max_rad <= 0.0f) ||
        (limits->velocity_max_rad_s <= 0.0f) ||
        (limits->torque_max_nm <= 0.0f) ||
        (J4310_Find(motor_id) != NULL))
    {
        return false;
    }

    for (index = 0U; index < J4310_MAX_MOTOR_COUNT; index++)
    {
        if (!j4310_context[index].used)
        {
            j4310_context[index].used = true;
            j4310_context[index].motor_id = motor_id;
            j4310_context[index].master_id = master_id;
            j4310_context[index].feedback_id = feedback_id;
            j4310_context[index].limits = *limits;
            return true;
        }
    }
    return false;
}

bool J4310_BuildEnable(uint8_t motor_id, can_frame_t *frame)
{
    return J4310_BuildSpecial(motor_id, J4310_CMD_ENABLE, frame);
}

bool J4310_BuildDisable(uint8_t motor_id, can_frame_t *frame)
{
    return J4310_BuildSpecial(motor_id, J4310_CMD_DISABLE, frame);
}

bool J4310_BuildMit(uint8_t motor_id,
                    float position_rad,
                    float velocity_rad_s,
                    float kp,
                    float kd,
                    float torque_nm,
                    can_frame_t *frame)
{
    j4310_context_t *context;
    uint16_t position;
    uint16_t velocity;
    uint16_t kp_raw;
    uint16_t kd_raw;
    uint16_t torque;

    context = J4310_Find(motor_id);
    if ((context == NULL) || (frame == NULL) ||
        !J4310_IsFinite(position_rad) ||
        !J4310_IsFinite(velocity_rad_s) || !J4310_IsFinite(kp) ||
        !J4310_IsFinite(kd) || !J4310_IsFinite(torque_nm))
    {
        return false;
    }

    position = J4310_FloatToUint(position_rad,
                                 -context->limits.position_max_rad,
                                 context->limits.position_max_rad,
                                 16U);
    velocity = J4310_FloatToUint(velocity_rad_s,
                                 -context->limits.velocity_max_rad_s,
                                 context->limits.velocity_max_rad_s,
                                 12U);
    kp_raw = J4310_FloatToUint(kp, 0.0f, J4310_KP_MAX, 12U);
    kd_raw = J4310_FloatToUint(kd, 0.0f, J4310_KD_MAX, 12U);
    torque = J4310_FloatToUint(torque_nm,
                               -context->limits.torque_max_nm,
                               context->limits.torque_max_nm,
                               12U);

    (void)memset(frame, 0, sizeof(*frame));
    frame->id = context->motor_id;
    frame->dlc = 8U;
    frame->data[0] = (uint8_t)(position >> 8U);
    frame->data[1] = (uint8_t)position;
    frame->data[2] = (uint8_t)(velocity >> 4U);
    frame->data[3] = (uint8_t)(((velocity & 0x0FU) << 4U) |
                               (kp_raw >> 8U));
    frame->data[4] = (uint8_t)kp_raw;
    frame->data[5] = (uint8_t)(kd_raw >> 4U);
    frame->data[6] = (uint8_t)(((kd_raw & 0x0FU) << 4U) |
                               (torque >> 8U));
    frame->data[7] = (uint8_t)torque;
    return true;
}

bool J4310_OnFrame(const can_frame_t *frame, uint32_t tick_ms)
{
    j4310_context_t *context;
    volatile j4310_feedback_t *feedback;
    uint16_t position;
    uint16_t velocity;
    uint16_t torque;
    uint32_t index;
    uint32_t sequence;

    if ((frame == NULL) || frame->extended || (frame->dlc != 8U))
    {
        return false;
    }

    context = NULL;
    for (index = 0U; index < J4310_MAX_MOTOR_COUNT; index++)
    {
        if (j4310_context[index].used &&
            (frame->id == j4310_context[index].master_id) &&
            ((frame->data[0] & 0x0FU) ==
             j4310_context[index].feedback_id))
        {
            context = &j4310_context[index];
            break;
        }
    }
    if (context == NULL)
    {
        return false;
    }

    position = (uint16_t)(((uint16_t)frame->data[1] << 8U) |
                          frame->data[2]);
    velocity = (uint16_t)(((uint16_t)frame->data[3] << 4U) |
                          (frame->data[4] >> 4U));
    torque = (uint16_t)((((uint16_t)frame->data[4] & 0x0FU) << 8U) |
                        frame->data[5]);

    feedback = &context->feedback;
    sequence = context->sequence;
    context->sequence = sequence + 1U;
    feedback->position_rad = J4310_UintToFloat(
        position, -context->limits.position_max_rad,
        context->limits.position_max_rad, 16U);
    feedback->velocity_rad_s = J4310_UintToFloat(
        velocity, -context->limits.velocity_max_rad_s,
        context->limits.velocity_max_rad_s, 12U);
    feedback->torque_nm = J4310_UintToFloat(
        torque, -context->limits.torque_max_nm,
        context->limits.torque_max_nm, 12U);
    feedback->mos_temperature_c = frame->data[6];
    feedback->rotor_temperature_c = frame->data[7];
    feedback->fault = frame->data[0] >> 4U;
    feedback->updated_at_ms = tick_ms;
    feedback->rx_frames++;
    context->sequence = sequence + 2U;
    return true;
}

bool J4310_GetFeedback(uint8_t motor_id, j4310_feedback_t *feedback)
{
    const j4310_context_t *context;
    uint32_t before;
    uint32_t after;

    context = J4310_Find(motor_id);
    if ((context == NULL) || (feedback == NULL))
    {
        return false;
    }

    for (;;)
    {
        before = context->sequence;
        if ((before & 1U) != 0U)
        {
            continue;
        }
        *feedback = context->feedback;
        after = context->sequence;
        if (before == after)
        {
            break;
        }
    }

    return feedback->rx_frames != 0U;
}
