#include "upper_pc_link.h"

#include <string.h>

#include "upper_config.h"

#define UPPER_ENABLE_ARM       (1U << 0)
#define UPPER_ENABLE_CONVEYOR  (1U << 1)
#define UPPER_ENABLE_GRIPPER   (1U << 2)
#define UPPER_STATE_PAYLOAD_SIZE 20U

static uint16_t UpperPcLink_ReadU16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t UpperPcLink_ReadU32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

static float UpperPcLink_ReadFloat(const uint8_t *data)
{
    uint32_t bits;
    float value;

    bits = UpperPcLink_ReadU32(data);
    (void)memcpy(&value, &bits, sizeof(value));
    return value;
}

static void UpperPcLink_WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static void UpperPcLink_WriteU32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

static bool UpperPcLink_DecodeTarget(const pc_frame_t *frame,
                                     upper_target_t *target)
{
    uint16_t enable_mask;
    const uint8_t *value;

    if ((frame == NULL) || (target == NULL) ||
        (frame->payload_len != UPPER_PC_CMD_PAYLOAD_SIZE))
    {
        return false;
    }

    (void)memset(target, 0, sizeof(*target));
    enable_mask = UpperPcLink_ReadU16(frame->payload);
    value = &frame->payload[2];

    target->arm.enabled = (enable_mask & UPPER_ENABLE_ARM) != 0U;
    target->conveyor.enabled = (enable_mask & UPPER_ENABLE_CONVEYOR) != 0U;
    target->gripper.enabled = (enable_mask & UPPER_ENABLE_GRIPPER) != 0U;

    target->arm.grip_pos_rad = UpperPcLink_ReadFloat(value + 0U);
    target->arm.grip_vel_rad_s = UpperPcLink_ReadFloat(value + 4U);
    target->arm.grip_kp = UpperPcLink_ReadFloat(value + 8U);
    target->arm.grip_kd = UpperPcLink_ReadFloat(value + 12U);
    target->arm.m3508_vel_rad_s[0] = UpperPcLink_ReadFloat(value + 16U);
    target->arm.m3508_vel_rad_s[1] = UpperPcLink_ReadFloat(value + 20U);
    target->conveyor.m2006_vel_rad_s = UpperPcLink_ReadFloat(value + 24U);
    target->gripper.m2006_vel_rad_s = UpperPcLink_ReadFloat(value + 28U);
    return true;
}

static void UpperPcLink_Accept(upper_pc_link_t *link,
                               const pc_frame_t *frame)
{
    link->last_rx_sequence = frame->sequence;
    link->last_rx_tick_ms = link->current_rx_tick_ms;
}

static void UpperPcLink_OnFrame(const pc_frame_t *frame, void *user_data)
{
    upper_pc_link_t *link;

    link = (upper_pc_link_t *)user_data;

    switch (frame->type)
    {
    case PC_MSG_HEARTBEAT:
        if (frame->payload_len == 0U)
        {
            UpperPcLink_Accept(link, frame);
        }
        break;

    case PC_MSG_ESTOP:
        if ((frame->payload_len == 1U) && (frame->payload[0] != 0U) &&
            (link->estop_handler != NULL))
        {
            UpperPcLink_Accept(link, frame);
            link->estop_handler(link->user_data);
        }
        break;

    case PC_MSG_UPPER_CMD:
    {
        upper_target_t target;

        if (UpperPcLink_DecodeTarget(frame, &target))
        {
            UpperPcLink_Accept(link, frame);
            link->remote_active = true;
            if (link->cmd_handler != NULL)
            {
                link->cmd_handler(&target, link->user_data);
            }
        }
        else
        {
            link->command_error_count++;
        }
        break;
    }

    default:
        break;
    }
}

void UpperPcLink_Init(upper_pc_link_t *link,
                      upper_pc_cmd_handler_t cmd_handler,
                      upper_pc_estop_handler_t estop_handler,
                      void *user_data)
{
    if (link == NULL)
    {
        return;
    }

    (void)memset(link, 0, sizeof(*link));
    PcProtocol_Init(&link->parser);
    link->cmd_handler = cmd_handler;
    link->estop_handler = estop_handler;
    link->user_data = user_data;
}

void UpperPcLink_Push(upper_pc_link_t *link,
                      const uint8_t *data,
                      size_t size,
                      uint32_t tick_ms)
{
    if ((link == NULL) || (data == NULL))
    {
        return;
    }

    link->current_rx_tick_ms = tick_ms;
    PcProtocol_Push(&link->parser, data, size, UpperPcLink_OnFrame, link);
}

bool UpperPcLink_IsTimedOut(const upper_pc_link_t *link, uint32_t tick_ms)
{
    if ((link == NULL) || !link->remote_active)
    {
        return false;
    }

    return (tick_ms - link->last_rx_tick_ms) > UPPER_PC_TIMEOUT_MS;
}

size_t UpperPcLink_BuildState(upper_pc_link_t *link,
                              const upper_robot_t *robot,
                              uint32_t tick_ms,
                              uint8_t *output,
                              size_t output_size)
{
    uint8_t payload[UPPER_STATE_PAYLOAD_SIZE];

    if ((link == NULL) || (robot == NULL))
    {
        return 0U;
    }

    payload[0] = (uint8_t)robot->state;
    payload[1] = link->remote_active ? 1U : 0U;
    UpperPcLink_WriteU32(&payload[2], tick_ms);
    UpperPcLink_WriteU16(&payload[6], link->last_rx_sequence);
    UpperPcLink_WriteU32(&payload[8], robot->motor_manager.sent_count);
    UpperPcLink_WriteU32(&payload[12], robot->motor_manager.send_fail_count);
    UpperPcLink_WriteU32(&payload[16],
                         robot->motor_manager.protocol_block_count);

    return PcProtocol_Encode(PC_MSG_ROBOT_STATE,
                             link->tx_sequence++,
                             payload,
                             sizeof(payload),
                             output,
                             output_size);
}
