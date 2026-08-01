#include "mg5010.h"

#include <float.h>
#include <limits.h>
#include <string.h>

#define MG5010_CAN_ID_BASE          0x140U
#define MG5010_FRAME_SIZE           8U
#define MG5010_REDUCTION_RATIO      36.0f
#define MG5010_CURRENT_A_PER_LSB    (66.0f / 4096.0f)
#define MG5010_CURRENT_RAW_LIMIT    2048
#define MG5010_RAD_TO_DEG           57.2957795131f
#define MG5010_DEG_TO_RAD           0.01745329252f

#define MG5010_CMD_RUN              0x88U
#define MG5010_CMD_STOP             0x81U
#define MG5010_CMD_STATUS_1         0x9AU
#define MG5010_CMD_CLEAR_ERROR      0x9BU
#define MG5010_CMD_STATUS_2         0x9CU
#define MG5010_CMD_CURRENT          0xA1U
#define MG5010_CMD_VELOCITY         0xA2U
#define MG5010_CMD_POSITION         0xA3U
#define MG5010_CMD_POSITION_SPEED   0xA4U
#define MG5010_CMD_MULTI_TURN       0x92U

typedef struct
{
    volatile uint32_t sequence;
    volatile mg5010_feedback_t feedback;
    int64_t software_zero_cdeg;
    volatile bool software_zero_valid;
} mg5010_context_t;

static mg5010_context_t mg5010_context[MG5010_MOTOR_ID_MAX];

static bool Mg5010_IsFinite(float value)
{
    return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX);
}

static bool Mg5010_IsValidId(uint8_t motor_id)
{
    return (motor_id >= MG5010_MOTOR_ID_MIN) &&
           (motor_id <= MG5010_MOTOR_ID_MAX);
}

static uint16_t Mg5010_ReadU16Le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static int16_t Mg5010_ReadI16Le(const uint8_t *data)
{
    return (int16_t)Mg5010_ReadU16Le(data);
}

static int64_t Mg5010_ReadI56Le(const uint8_t *data)
{
    uint64_t value;
    uint32_t index;

    value = 0U;
    for (index = 0U; index < 7U; index++)
    {
        value |= (uint64_t)data[index] << (index * 8U);
    }
    if ((value & (1ULL << 55U)) != 0U)
    {
        value |= 0xFF00000000000000ULL;
    }
    return (int64_t)value;
}

static void Mg5010_WriteU16Le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static void Mg5010_WriteU32Le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

static int32_t Mg5010_RoundI32(float value)
{
    return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static bool Mg5010_PrepareFrame(uint8_t motor_id,
                                uint8_t command,
                                can_frame_t *frame)
{
    if (!Mg5010_IsValidId(motor_id) || (frame == NULL))
    {
        return false;
    }

    (void)memset(frame, 0, sizeof(*frame));
    frame->id = MG5010_CAN_ID_BASE + motor_id;
    frame->dlc = MG5010_FRAME_SIZE;
    frame->data[0] = command;
    return true;
}

static bool Mg5010_CurrentToRaw(float current_a, int16_t *raw)
{
    int32_t scaled;

    if ((raw == NULL) || !Mg5010_IsFinite(current_a))
    {
        return false;
    }

    scaled = Mg5010_RoundI32(current_a / MG5010_CURRENT_A_PER_LSB);
    if ((scaled < -MG5010_CURRENT_RAW_LIMIT) ||
        (scaled > MG5010_CURRENT_RAW_LIMIT))
    {
        return false;
    }

    *raw = (int16_t)scaled;
    return true;
}

void Mg5010_Init(void)
{
    uint32_t index;

    (void)memset(mg5010_context, 0, sizeof(mg5010_context));
    for (index = 0U; index < MG5010_MOTOR_ID_MAX; index++)
    {
        mg5010_context[index].feedback.motor_id = (uint8_t)(index + 1U);
    }
}

bool Mg5010_BuildRun(uint8_t motor_id, can_frame_t *frame)
{
    return Mg5010_PrepareFrame(motor_id, MG5010_CMD_RUN, frame);
}

bool Mg5010_BuildStop(uint8_t motor_id, can_frame_t *frame)
{
    return Mg5010_PrepareFrame(motor_id, MG5010_CMD_STOP, frame);
}

bool Mg5010_BuildReadPosition(uint8_t motor_id, can_frame_t *frame)
{
    return Mg5010_PrepareFrame(motor_id, MG5010_CMD_MULTI_TURN, frame);
}

bool Mg5010_BuildCurrent(uint8_t motor_id,
                         float current_a,
                         can_frame_t *frame)
{
    int16_t current_raw;

    if (!Mg5010_PrepareFrame(motor_id, MG5010_CMD_CURRENT, frame) ||
        !Mg5010_CurrentToRaw(current_a, &current_raw))
    {
        return false;
    }

    Mg5010_WriteU16Le(&frame->data[4], (uint16_t)current_raw);
    return true;
}

bool Mg5010_BuildVelocity(uint8_t motor_id,
                          float output_vel_rad_s,
                          float current_limit_a,
                          can_frame_t *frame)
{
    float motor_speed_cdeg_s;
    int16_t current_raw;
    int32_t speed_raw;

    motor_speed_cdeg_s = output_vel_rad_s * MG5010_RAD_TO_DEG *
                         MG5010_REDUCTION_RATIO * 100.0f;
    if (!Mg5010_PrepareFrame(motor_id, MG5010_CMD_VELOCITY, frame) ||
        !Mg5010_CurrentToRaw(current_limit_a, &current_raw) ||
        !Mg5010_IsFinite(motor_speed_cdeg_s) ||
        (motor_speed_cdeg_s < (float)INT32_MIN) ||
        (motor_speed_cdeg_s > (float)INT32_MAX))
    {
        return false;
    }

    speed_raw = Mg5010_RoundI32(motor_speed_cdeg_s);
    Mg5010_WriteU16Le(&frame->data[2], (uint16_t)current_raw);
    Mg5010_WriteU32Le(&frame->data[4], (uint32_t)speed_raw);
    return true;
}

bool Mg5010_BuildPosition(uint8_t motor_id,
                          float output_pos_rad,
                          float max_output_vel_rad_s,
                          can_frame_t *frame)
{
    float motor_angle_cdeg;
    float motor_speed_dps;
    int64_t target_cdeg;
    int32_t angle_raw;
    uint8_t command;
    mg5010_context_t *context;

    if (!Mg5010_IsValidId(motor_id))
    {
        return false;
    }
    context = &mg5010_context[motor_id - 1U];
    if (!context->software_zero_valid)
    {
        return false;
    }

    motor_angle_cdeg = output_pos_rad * MG5010_RAD_TO_DEG *
                       MG5010_REDUCTION_RATIO * 100.0f;
    motor_speed_dps = max_output_vel_rad_s * MG5010_RAD_TO_DEG *
                      MG5010_REDUCTION_RATIO;
    command = (motor_speed_dps > 0.0f) ? MG5010_CMD_POSITION_SPEED :
                                        MG5010_CMD_POSITION;

    if (!Mg5010_IsFinite(motor_angle_cdeg) ||
        !Mg5010_IsFinite(motor_speed_dps) ||
        (motor_angle_cdeg < (float)INT32_MIN) ||
        (motor_angle_cdeg > (float)INT32_MAX) ||
        (motor_speed_dps < 0.0f) ||
        (motor_speed_dps > 65535.0f) ||
        (frame == NULL))
    {
        return false;
    }

    target_cdeg = context->software_zero_cdeg +
                  (int64_t)Mg5010_RoundI32(motor_angle_cdeg);
    if ((target_cdeg < INT32_MIN) || (target_cdeg > INT32_MAX) ||
        !Mg5010_PrepareFrame(motor_id, command, frame))
    {
        return false;
    }

    angle_raw = (int32_t)target_cdeg;
    if (command == MG5010_CMD_POSITION_SPEED)
    {
        Mg5010_WriteU16Le(&frame->data[2],
                          (uint16_t)(motor_speed_dps + 0.5f));
    }
    Mg5010_WriteU32Le(&frame->data[4], (uint32_t)angle_raw);
    return true;
}

bool Mg5010_OnFrame(uint8_t motor_id,
                     const can_frame_t *frame,
                     uint32_t tick_ms)
{
    mg5010_context_t *context;
    volatile mg5010_feedback_t *feedback;
    uint8_t command;
    int64_t angle_cdeg;
    uint32_t sequence;

    if (!Mg5010_IsValidId(motor_id) || (frame == NULL) ||
        frame->extended || (frame->dlc != 8U) ||
        (frame->id != (MG5010_CAN_ID_BASE + motor_id)))
    {
        return false;
    }

    context = &mg5010_context[motor_id - 1U];
    feedback = &context->feedback;
    command = frame->data[0];
    sequence = context->sequence;
    context->sequence = sequence + 1U;

    feedback->motor_id = motor_id;
    feedback->command = command;
    feedback->updated_at_ms = tick_ms;
    feedback->rx_frames++;

    switch (command)
    {
    case MG5010_CMD_STATUS_1:
    case MG5010_CMD_CLEAR_ERROR:
        feedback->temperature_c = (int8_t)frame->data[1];
        feedback->bus_voltage_v =
            (float)Mg5010_ReadU16Le(&frame->data[2]) * 0.01f;
        feedback->bus_current_a =
            (float)Mg5010_ReadI16Le(&frame->data[4]) * 0.01f;
        feedback->motor_state = frame->data[6];
        feedback->error_state = frame->data[7];
        break;

    case MG5010_CMD_STATUS_2:
    case MG5010_CMD_CURRENT:
    case MG5010_CMD_VELOCITY:
    case MG5010_CMD_POSITION:
    case MG5010_CMD_POSITION_SPEED:
        feedback->temperature_c = (int8_t)frame->data[1];
        feedback->torque_current_a =
            (float)Mg5010_ReadI16Le(&frame->data[2]) *
            MG5010_CURRENT_A_PER_LSB;
        feedback->output_vel_rad_s =
            (float)Mg5010_ReadI16Le(&frame->data[4]) *
            MG5010_DEG_TO_RAD / MG5010_REDUCTION_RATIO;
        feedback->encoder = Mg5010_ReadU16Le(&frame->data[6]);
        break;

    case MG5010_CMD_MULTI_TURN:
        angle_cdeg = Mg5010_ReadI56Le(&frame->data[1]);
        if (!context->software_zero_valid)
        {
            context->software_zero_cdeg = angle_cdeg;
            context->software_zero_valid = true;
        }
        feedback->output_pos_rad =
            (float)(angle_cdeg - context->software_zero_cdeg) * 0.01f *
                                   MG5010_DEG_TO_RAD /
                                   MG5010_REDUCTION_RATIO;
        feedback->output_pos_valid = true;
        break;

    case MG5010_CMD_RUN:
    case MG5010_CMD_STOP:
        break;

    default:
        context->sequence = sequence + 2U;
        return false;
    }

    context->sequence = sequence + 2U;
    return true;
}

bool Mg5010_PositionReady(uint8_t motor_id)
{
    if (!Mg5010_IsValidId(motor_id))
    {
        return false;
    }
    return mg5010_context[motor_id - 1U].software_zero_valid;
}

bool Mg5010_GetFeedback(uint8_t motor_id, mg5010_feedback_t *feedback)
{
    const mg5010_context_t *context;
    uint32_t before;
    uint32_t after;

    if (!Mg5010_IsValidId(motor_id) || (feedback == NULL))
    {
        return false;
    }

    context = &mg5010_context[motor_id - 1U];
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
