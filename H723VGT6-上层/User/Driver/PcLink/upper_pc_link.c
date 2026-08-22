/**
 * @file upper_pc_link.c
 * @brief 实现上层控制板与 PC 之间的命令解析、会话管理和状态回传。
 */

#include "upper_pc_link.h"

#include <string.h>

#define UPPER_ENABLE_ARM       (1U << 0)
#define UPPER_ENABLE_GATE      (1U << 1)
#define UPPER_ENABLE_GRIPPER   (1U << 2)
#define UPPER_ENABLE_DJI_SYNC  (1U << 3)
#define UPPER_ENABLE_J4310_ONLY (1U << 4)
#define UPPER_ENABLE_M3508_ONLY (1U << 5)
#define UPPER_COMMAND_J4310_STOP (1U << 6)
#define UPPER_STATE_BASE_PAYLOAD_SIZE 84U
#define UPPER_STATE_DJI_DIAGNOSTIC_SIZE 16U
#define UPPER_DJI_TELEMETRY_PAYLOAD_SIZE \
    (UPPER_STATE_DJI_DIAGNOSTIC_SIZE + \
     UPPER_PC_FDCAN_COUNT * sizeof(uint32_t))
#define UPPER_DJI_FLAG_FEEDBACK_RECEIVED (1U << 0)
#define UPPER_DJI_FLAG_ZERO_VALID        (1U << 1)
#define UPPER_DJI_FLAG_FEEDBACK_FRESH    (1U << 2)
#define UPPER_MOTOR_EVENT_PAYLOAD_SIZE 8U
#define UPPER_PC_RPM_TO_RAD_S    0.10471975512f
#define UPPER_PC_RAD_S_TO_RPM    9.54929658551f
#define UPPER_PC_M3508_CURRENT_SCALE (20.0f / 16384.0f)
#define UPPER_PC_M2006_CURRENT_SCALE (10.0f / 10000.0f)
#define UPPER_PC_LINK_WATCHDOGS_ENABLED 0U
#define UPPER_PC_TIMEOUT_MS            200U
#define UPPER_PC_J4310_TORQUE_LIMIT_NM  10.0f
static const uint8_t UPPER_PC_HANDSHAKE_MAGIC[UPPER_PC_HANDSHAKE_PAYLOAD_SIZE] =
{
    'H', '7', '2', '3'
};

/* 功能：读取小端 16 位整数；用途：解析上位机载荷字段；返回值表示解码后的无符号数。 */
static uint16_t UpperPcLink_ReadU16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

/* 功能：读取小端 32 位整数；用途：解析上位机计数和时间字段；返回值表示解码后的无符号数。 */
static uint32_t UpperPcLink_ReadU32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

/* 功能：按小端位模式读取单精度浮点数；用途：解析上位机控制量；返回值表示还原后的 float。 */
static float UpperPcLink_ReadFloat(const uint8_t *data)
{
    uint32_t bits;
    float value;

    bits = UpperPcLink_ReadU32(data);
    (void)memcpy(&value, &bits, sizeof(value));
    return value;
}

/* 功能：将 16 位整数写成小端字节；用途：编码上位机协议字段；无返回值表示结果写入 data。 */
static void UpperPcLink_WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

/* 功能：将 32 位整数写成小端字节；用途：编码计数和时间字段；无返回值表示结果写入 data。 */
static void UpperPcLink_WriteU32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

/* 功能：按小端位模式写入单精度浮点数；用途：编码状态和遥测量；无返回值表示结果写入 data。 */
static void UpperPcLink_WriteFloat(uint8_t *data, float value)
{
    uint32_t bits;

    (void)memcpy(&bits, &value, sizeof(bits));
    UpperPcLink_WriteU32(data, bits);
}

/* 功能：读取 GUI 下发的 PID 参数并换算到固件单位；用途：适配 M3508/M2006 的速度环或位置环；结果写入 cfg。 */
static void UpperPcLink_ReadGuiPid(const uint8_t *data,
                                   upper_pc_pid_cfg_t *cfg,
                                   bool m3508,
                                   bool speed_loop)
{
    float current_scale;

    current_scale = m3508 ? UPPER_PC_M3508_CURRENT_SCALE :
                             UPPER_PC_M2006_CURRENT_SCALE;
    if (speed_loop)
    {
        cfg->kp = UpperPcLink_ReadFloat(data + 0U) * current_scale *
                  UPPER_PC_RAD_S_TO_RPM;
        cfg->ki = UpperPcLink_ReadFloat(data + 4U) * current_scale *
                  UPPER_PC_RAD_S_TO_RPM;
        cfg->kd = UpperPcLink_ReadFloat(data + 8U) * current_scale *
                  UPPER_PC_RAD_S_TO_RPM;
        cfg->integral_limit = UpperPcLink_ReadFloat(data + 12U) *
                              UPPER_PC_RPM_TO_RAD_S;
        cfg->output_limit = UpperPcLink_ReadFloat(data + 16U) * current_scale;
    }
    else
    {
        cfg->kp = UpperPcLink_ReadFloat(data + 0U) * UPPER_PC_RPM_TO_RAD_S;
        cfg->ki = UpperPcLink_ReadFloat(data + 4U) * UPPER_PC_RPM_TO_RAD_S;
        cfg->kd = UpperPcLink_ReadFloat(data + 8U) * UPPER_PC_RPM_TO_RAD_S;
        cfg->integral_limit = UpperPcLink_ReadFloat(data + 12U);
        cfg->output_limit = UpperPcLink_ReadFloat(data + 16U) *
                            UPPER_PC_RPM_TO_RAD_S;
    }
}

/* 功能：把上位机命令帧解码为整机目标；用途：解析使能位、模式、运动量和可选 PID；返回 true 表示帧合法。 */
static bool UpperPcLink_DecodeTarget(const pc_frame_t *frame,
                                     upper_pc_target_t *target)
{
    uint16_t enable_mask;
    const uint8_t *value;
    bool extended;
    bool position_mode;

    if ((frame == NULL) || (target == NULL) ||
        ((frame->type != PC_MSG_UPPER_CMD) &&
         (frame->type != PC_MSG_UPPER_POSITION_CMD)) ||
        ((frame->payload_len != UPPER_PC_CMD_PAYLOAD_SIZE) &&
         (frame->payload_len != UPPER_PC_POSITION_TORQUE_CMD_PAYLOAD_SIZE) &&
         (frame->payload_len != UPPER_PC_EXTENDED_POSITION_CMD_PAYLOAD_SIZE)))
    {
        return false;
    }

    (void)memset(target, 0, sizeof(*target));
    position_mode = frame->type == PC_MSG_UPPER_POSITION_CMD;
    extended = frame->payload_len == UPPER_PC_EXTENDED_POSITION_CMD_PAYLOAD_SIZE;
    if ((frame->payload_len != UPPER_PC_CMD_PAYLOAD_SIZE) && !position_mode)
    {
        return false;
    }
    target->position_mode = position_mode;
    enable_mask = UpperPcLink_ReadU16(frame->payload);
    value = &frame->payload[2];

    /* 扩展命令共用一组 C610 PID 参数，同时选择两个机构会产生歧义。
     * 紧凑位置命令为每个槽位提供独立目标，因此可以安全地同时控制两者。 */
    if (((enable_mask & UPPER_ENABLE_GATE) != 0U) &&
        ((enable_mask & UPPER_ENABLE_GRIPPER) != 0U) &&
        (frame->payload_len != UPPER_PC_POSITION_TORQUE_CMD_PAYLOAD_SIZE))
    {
        return false;
    }

    target->arm.enabled = ((enable_mask & UPPER_ENABLE_ARM) != 0U) ||
                          ((enable_mask & UPPER_ENABLE_J4310_ONLY) != 0U);
    target->arm.j4310_commanded =
        target->arm.enabled ||
        ((enable_mask & UPPER_COMMAND_J4310_STOP) != 0U);
    target->arm.m3508_enabled = ((enable_mask & UPPER_ENABLE_ARM) != 0U) ||
                                ((enable_mask & UPPER_ENABLE_DJI_SYNC) != 0U) ||
                                ((enable_mask & UPPER_ENABLE_M3508_ONLY) != 0U);
    target->gate.enabled =
        (enable_mask & UPPER_ENABLE_GATE) != 0U;
    target->gripper.enabled =
        (enable_mask & UPPER_ENABLE_GRIPPER) != 0U;
    target->arm.position_mode = position_mode;
    target->gate.position_mode = position_mode;
    target->gripper.position_mode = position_mode;
    target->arm.pid_update = extended &&
                             (target->arm.enabled ||
                              target->arm.m3508_enabled);
    target->gate.pid_update = extended && target->gate.enabled;
    target->gripper.pid_update = extended && target->gripper.enabled;

    target->arm.grip_pos_rad = UpperPcLink_ReadFloat(value + 0U);
    target->arm.grip_vel_rad_s = UpperPcLink_ReadFloat(value + 4U);
    target->arm.grip_kp = UpperPcLink_ReadFloat(value + 8U);
    target->arm.grip_kd = UpperPcLink_ReadFloat(value + 12U);
    target->arm.grip_torque_nm =
        (frame->payload_len != UPPER_PC_CMD_PAYLOAD_SIZE) ?
        UpperPcLink_ReadFloat(value + 16U) : 0.0f;
    target->arm.grip_torque_limit_nm =
        (frame->payload_len != UPPER_PC_CMD_PAYLOAD_SIZE) ?
                                       UpperPcLink_ReadFloat(value + 20U) :
                                       UPPER_PC_J4310_TORQUE_LIMIT_NM;
    if (frame->payload_len != UPPER_PC_CMD_PAYLOAD_SIZE)
    {
        target->arm.m3508_pos_rad[0] = UpperPcLink_ReadFloat(value + 24U);
        target->arm.m3508_pos_rad[1] = UpperPcLink_ReadFloat(value + 28U);
        target->gate.m2006_pos_rad = UpperPcLink_ReadFloat(value + 32U);
        target->gripper.m2006_pos_rad = UpperPcLink_ReadFloat(value + 36U);
        if (extended)
        {
            UpperPcLink_ReadGuiPid(value + 40U,
                                   &target->arm.m3508_speed_pid,
                                   true,
                                   true);
            UpperPcLink_ReadGuiPid(value + 60U,
                                   &target->arm.m3508_position_pid,
                                   true,
                                   false);
            UpperPcLink_ReadGuiPid(value + 80U,
                                   &target->gate.m2006_speed_pid,
                                   false,
                                   true);
            UpperPcLink_ReadGuiPid(value + 100U,
                                   &target->gate.m2006_position_pid,
                                   false,
                                   false);
                target->gripper.m2006_speed_pid =
                target->gate.m2006_speed_pid;
                target->gripper.m2006_position_pid =
                target->gate.m2006_position_pid;
        }
    }
    else if (position_mode)
    {
        target->arm.m3508_pos_rad[0] = UpperPcLink_ReadFloat(value + 16U);
        target->arm.m3508_pos_rad[1] = UpperPcLink_ReadFloat(value + 20U);
        target->gate.m2006_pos_rad = UpperPcLink_ReadFloat(value + 24U);
        target->gripper.m2006_pos_rad = UpperPcLink_ReadFloat(value + 28U);
    }
    else
    {
        target->arm.m3508_vel_rad_s[0] = UpperPcLink_ReadFloat(value + 16U);
        target->arm.m3508_vel_rad_s[1] = UpperPcLink_ReadFloat(value + 20U);
        target->gate.m2006_vel_rad_s = UpperPcLink_ReadFloat(value + 24U);
        target->gripper.m2006_vel_rad_s = UpperPcLink_ReadFloat(value + 28U);
    }
    return true;
}

/* 功能：记录一帧已接受消息的序号和接收时间；用途：刷新会话心跳与超时基准；无返回值表示链路状态已更新。 */
static void UpperPcLink_Accept(upper_pc_link_t *link,
                               const pc_frame_t *frame)
{
    link->last_rx_sequence = frame->sequence;
    link->last_rx_tick_ms = link->current_rx_tick_ms;
}

/* 功能：分发一帧已解析的上位机消息；用途：处理握手、心跳、急停、目标和电机动作；业务结果通过已注册回调上报。 */
static void UpperPcLink_OnFrame(const pc_frame_t *frame, void *user_data)
{
    upper_pc_link_t *link;

    link = (upper_pc_link_t *)user_data;

    switch (frame->type)
    {
    case PC_MSG_HANDSHAKE:
        if ((frame->payload_len == UPPER_PC_HANDSHAKE_PAYLOAD_SIZE) &&
            (memcmp(frame->payload,
                    UPPER_PC_HANDSHAKE_MAGIC,
                    UPPER_PC_HANDSHAKE_PAYLOAD_SIZE) == 0))
        {
            UpperPcLink_Accept(link, frame);
            if ((UPPER_PC_LINK_WATCHDOGS_ENABLED != 0U) &&
                link->remote_active)
            {
                link->remote_active = false;
                link->remote_timeout_pending = true;
            }
            link->session_active = false;
            link->handshake_sequence = frame->sequence;
            link->handshake_received_tick_ms = link->current_rx_tick_ms;
            link->handshake_received_count++;
            link->handshake_pending = true;
            link->flash_info_pending = false;
        }
        break;

    case PC_MSG_HEARTBEAT:
        if ((frame->payload_len == 0U) &&
            UpperPcLink_IsSessionActive(link,
                                        link->current_rx_tick_ms))
        {
            UpperPcLink_Accept(link, frame);
        }
        break;

    case PC_MSG_ESTOP:
        if ((frame->payload_len == 1U) && (frame->payload[0] != 0U) &&
            UpperPcLink_IsSessionActive(link,
                                        link->current_rx_tick_ms) &&
            (link->estop_handler != NULL))
        {
            UpperPcLink_Accept(link, frame);
            link->estop_handler(link->user_data);
            link->remote_active = false;
        }
        break;

    case PC_MSG_UPPER_CMD:
    case PC_MSG_UPPER_POSITION_CMD:
    {
        upper_pc_target_t target;

        if (!UpperPcLink_IsSessionActive(link,
                                         link->current_rx_tick_ms))
        {
            break;
        }
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

    case PC_MSG_MOTOR_ACTION:
        if (!UpperPcLink_IsSessionActive(link,
                                         link->current_rx_tick_ms))
        {
            break;
        }
        if ((link->motor_action_handler != NULL) &&
            (((frame->payload_len == UPPER_PC_MOTOR_ACTION_PAYLOAD_SIZE) &&
              ((frame->payload[0] == UPPER_PC_ACTION_J4310_SAVE_ZERO) ||
               (frame->payload[0] == UPPER_PC_ACTION_J4310_ENABLE))) ||
             ((frame->payload_len ==
               UPPER_PC_MOTOR_CONFIG_ACTION_PAYLOAD_SIZE) &&
              (frame->payload[0] == UPPER_PC_ACTION_J4310_AUTO_RETURN) &&
              (frame->payload[3] <= 1U))))
        {
            UpperPcLink_Accept(link, frame);
            link->motor_action_handler(frame->payload[0],
                                       frame->payload[1],
                                       frame->payload[2],
                                       (frame->payload_len ==
                                        UPPER_PC_MOTOR_CONFIG_ACTION_PAYLOAD_SIZE) ?
                                       frame->payload[3] : 0U,
                                       link->user_data);
        }
        else
        {
            link->command_error_count++;
        }
        break;

    case PC_MSG_AUX_CONTROL:
        if (!UpperPcLink_IsSessionActive(link,
                                         link->current_rx_tick_ms))
        {
            break;
        }
        if ((frame->payload_len == UPPER_PC_AUX_CONTROL_PAYLOAD_SIZE) &&
            ((frame->payload[0] & 0xF0U) == 0U) &&
            ((frame->payload[1] & 0xF0U) == 0U) &&
            (link->aux_control_handler != NULL))
        {
            uint8_t update_mask = frame->payload[1] & 0x0FU;

            UpperPcLink_Accept(link, frame);
            link->aux_control_handler(frame->payload[0],
                                      update_mask,
                                      link->user_data);
        }
        else
        {
            link->command_error_count++;
        }
        break;

    case PC_MSG_FLASH_INFO_REQUEST:
        if (frame->payload_len == 0U)
        {
            link->flash_info_sequence = frame->sequence;
            link->flash_info_pending = true;
        }
        else
        {
            link->command_error_count++;
        }
        break;

    default:
        break;
    }
}

/* 功能：初始化上位机链路和业务回调；用途：建立协议解析与会话状态；无返回值表示 link 被重置并绑定处理器。 */
void UpperPcLink_Init(upper_pc_link_t *link,
                      upper_pc_cmd_handler_t cmd_handler,
                      upper_pc_estop_handler_t estop_handler,
                      upper_pc_motor_action_handler_t motor_action_handler,
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
    link->motor_action_handler = motor_action_handler;
    link->user_data = user_data;
}

/* 注册辅助控制命令回调；未注册时命令不会触发业务动作。 */
void UpperPcLink_SetAuxControlHandler(upper_pc_link_t *link,
                                      upper_pc_aux_control_handler_t handler)
{
    if (link != NULL)
    {
        link->aux_control_handler = handler;
    }
}

/* 功能：向链路输入一段接收数据；用途：更新时间并驱动流式协议解析；完整消息会自动进入帧处理函数。 */
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
    (void)UpperPcLink_IsSessionActive(link, tick_ms);
    PcProtocol_Push(&link->parser, data, size, UpperPcLink_OnFrame, link);
}

/* 功能：检查并消费遥控链路超时事件；用途：通知控制层停止正在运行的机器人；返回 true 表示本次检测到超时。 */
bool UpperPcLink_IsTimedOut(upper_pc_link_t *link, uint32_t tick_ms)
{
    if (link == NULL)
    {
        return false;
    }

    if (UPPER_PC_LINK_WATCHDOGS_ENABLED == 0U)
    {
        link->remote_timeout_pending = false;
        (void)tick_ms;
        return false;
    }

    if (link->remote_timeout_pending)
    {
        link->remote_timeout_pending = false;
        return true;
    }
    if (link->remote_active &&
        ((tick_ms - link->last_rx_tick_ms) > UPPER_PC_TIMEOUT_MS))
    {
        link->remote_active = false;
        link->session_active = false;
        return true;
    }
    return false;
}

/* 功能：检查握手会话当前是否有效；用途：拒绝未握手或已超时的控制命令；返回 true 表示会话仍在有效期内。 */
bool UpperPcLink_IsSessionActive(upper_pc_link_t *link, uint32_t tick_ms)
{
    if ((link == NULL) || !link->session_active)
    {
        return false;
    }

    if (UPPER_PC_LINK_WATCHDOGS_ENABLED == 0U)
    {
        (void)tick_ms;
        return true;
    }

    if ((tick_ms - link->last_rx_tick_ms) > UPPER_PC_TIMEOUT_MS)
    {
        link->session_active = false;
        link->remote_timeout_pending = link->remote_active;
        link->remote_active = false;
        return false;
    }
    return true;
}

/* 功能：查询是否有握手确认待发送；用途：驱动确认帧发送流程；返回 true 表示存在待确认握手。 */
bool UpperPcLink_HasHandshakePending(const upper_pc_link_t *link)
{
    return (link != NULL) && link->handshake_pending;
}

/* 功能：判断握手确认是否已达到保护延时；用途：控制 ACK 的最早发送时刻；返回 true 表示现在可以发送。 */
bool UpperPcLink_IsHandshakeAckDue(const upper_pc_link_t *link,
                                   uint32_t tick_ms,
                                   uint32_t guard_ms)
{
    return (link != NULL) && link->handshake_pending &&
           ((tick_ms - link->handshake_received_tick_ms) >= guard_ms);
}

/* 功能：取得待确认握手的序号；用途：匹配 ACK 与请求；返回值为序号，空链路时返回 0。 */
uint16_t UpperPcLink_GetHandshakeSequence(const upper_pc_link_t *link)
{
    return (link != NULL) ? link->handshake_sequence : 0U;
}

/* 功能：构造握手确认协议帧；用途：向上位机确认 H723 会话；返回 0 表示无待确认握手或构帧失败。 */
size_t UpperPcLink_BuildHandshakeAck(const upper_pc_link_t *link,
                                     uint8_t *output,
                                     size_t output_size)
{
    if ((link == NULL) || !link->handshake_pending)
    {
        return 0U;
    }

    return PcProtocol_Encode(PC_MSG_ACK,
                             link->handshake_sequence,
                             UPPER_PC_HANDSHAKE_MAGIC,
                             UPPER_PC_HANDSHAKE_PAYLOAD_SIZE,
                             output,
                             output_size);
}

/* 功能：标记指定握手 ACK 已发送；用途：结束等待并激活会话；仅序号匹配时状态才改变。 */
void UpperPcLink_MarkHandshakeAckSent(upper_pc_link_t *link,
                                      uint16_t sequence)
{
    if ((link != NULL) && link->handshake_pending &&
        (link->handshake_sequence == sequence))
    {
        link->handshake_pending = false;
        link->session_active = true;
    }
}

/* Flash 信息仅针对已接受的一次性请求发送。 */
/* 功能：检查是否有待回传的 Flash 信息；用途：决定通信任务是否需要发送查询结果；返回 true 表示存在待发信息。 */
bool UpperPcLink_HasFlashInfoPending(const upper_pc_link_t *link)
{
    return (link != NULL) && link->flash_info_pending;
}

/* 功能：读取 Flash 信息请求的序列号；用途：让响应帧与请求正确对应；返回值表示待响应序列号。 */
uint16_t UpperPcLink_GetFlashInfoSequence(const upper_pc_link_t *link)
{
    return (link != NULL) ? link->flash_info_sequence : 0U;
}

/* 功能：构造外部 Flash 信息响应帧；用途：向 PC 回传初始化状态、容量和器件标识；返回值表示完整帧长度。 */
size_t UpperPcLink_BuildFlashInfo(const upper_pc_link_t *link,
                                  uint8_t init_status,
                                  bool initialized,
                                  uint32_t jedec_id,
                                  uint32_t capacity_kb,
                                  uint32_t sector_count,
                                  uint16_t page_size_byte,
                                  uint32_t sector_size_byte,
                                  uint8_t *output,
                                  size_t output_size)
{
    uint8_t payload[UPPER_PC_FLASH_INFO_PAYLOAD_SIZE];

    if ((link == NULL) || !link->flash_info_pending)
    {
        return 0U;
    }

    payload[0] = init_status;
    payload[1] = initialized ? 1U : 0U;
    UpperPcLink_WriteU32(&payload[2], jedec_id);
    UpperPcLink_WriteU32(&payload[6], capacity_kb);
    UpperPcLink_WriteU32(&payload[10], sector_count);
    UpperPcLink_WriteU16(&payload[14], page_size_byte);
    UpperPcLink_WriteU32(&payload[16], sector_size_byte);
    return PcProtocol_Encode(PC_MSG_FLASH_INFO,
                             link->flash_info_sequence,
                             payload,
                             sizeof(payload),
                             output,
                             output_size);
}

/* 功能：确认指定 Flash 信息响应已经发送；用途：清除一次性待发送标志；无返回值表示链路状态已更新。 */
void UpperPcLink_MarkFlashInfoSent(upper_pc_link_t *link,
                                   uint16_t sequence)
{
    if ((link != NULL) && link->flash_info_pending &&
        (link->flash_info_sequence == sequence))
    {
        link->flash_info_pending = false;
    }
}

/* 功能：构造机器人基础状态帧；用途：上报状态、链路、发送统计和 J4310 位置；返回值表示完整帧长度。 */
size_t UpperPcLink_BuildState(upper_pc_link_t *link,
                              const upper_pc_state_t *state,
                              uint32_t tick_ms,
                              uint8_t *output,
                              size_t output_size)
{
    uint8_t payload[UPPER_STATE_BASE_PAYLOAD_SIZE];

    if ((link == NULL) || (state == NULL))
    {
        return 0U;
    }

    (void)memset(payload, 0, sizeof(payload));
    payload[0] = state->robot_state;
    payload[1] = link->remote_active ? 1U : 0U;
    UpperPcLink_WriteU32(&payload[2], tick_ms);
    UpperPcLink_WriteU16(&payload[6], link->last_rx_sequence);
    UpperPcLink_WriteU32(&payload[8], state->motor_sent_count);
    UpperPcLink_WriteU32(&payload[12], state->motor_send_fail_count);
    UpperPcLink_WriteU32(&payload[16], state->motor_protocol_block_count);
    UpperPcLink_WriteFloat(&payload[20], state->j4310_position_rad);
    payload[24] = state->j4310_position_valid ? 1U : 0U;
    UpperPcLink_WriteU32(&payload[25], state->j4310_bus_rx_frames);
    if (state->j4310_rx_valid)
    {
        const upper_pc_j4310_rx_diagnostic_t *j4310_rx = &state->j4310_rx;

        UpperPcLink_WriteU32(&payload[29], j4310_rx->accepted_frames);
        UpperPcLink_WriteU32(&payload[33],
                             j4310_rx->rejected_format_frames);
        UpperPcLink_WriteU32(&payload[37],
                             j4310_rx->rejected_master_id_frames);
        UpperPcLink_WriteU32(&payload[41],
                             j4310_rx->rejected_feedback_id_frames);
        UpperPcLink_WriteU16(&payload[45], j4310_rx->last_can_id);
        payload[47] = j4310_rx->last_dlc;
        payload[48] = j4310_rx->last_data0;
        payload[49] = j4310_rx->last_result;
    }
    if (state->j4310_tx_valid)
    {
        const upper_pc_j4310_tx_diagnostic_t *j4310_tx = &state->j4310_tx;

        UpperPcLink_WriteU32(&payload[50], j4310_tx->attempted_frames);
        UpperPcLink_WriteU32(&payload[54], j4310_tx->queued_frames);
        UpperPcLink_WriteU32(&payload[58], j4310_tx->failed_frames);
        UpperPcLink_WriteU32(&payload[62], j4310_tx->enable_frames);
        UpperPcLink_WriteU32(&payload[66], j4310_tx->mit_frames);
        UpperPcLink_WriteU32(&payload[70], j4310_tx->disable_frames);
        UpperPcLink_WriteU16(&payload[74], j4310_tx->last_can_id);
        payload[76] = j4310_tx->last_dlc;
        payload[77] = j4310_tx->last_data7;
        payload[78] = j4310_tx->enable_confirmed ? 1U : 0U;
        payload[79] = j4310_tx->feedback_state;
    }
    payload[80] = state->j4310_auto_return.available ? 1U : 0U;
    payload[81] = state->j4310_auto_return.enabled ? 1U : 0U;
    payload[82] = state->j4310_auto_return.active ? 1U : 0U;
    payload[83] = state->j4310_auto_return.stage;
    return PcProtocol_Encode(PC_MSG_ROBOT_STATE,
                             link->tx_sequence++,
                             payload,
                             sizeof(payload),
                             output,
                             output_size);
}

/* 功能：按统一格式构造电机事件帧；用途：复用动作结果与故障消息的公共编码；返回值表示完整帧长度。 */
static size_t UpperPcLink_BuildMotorEvent(upper_pc_link_t *link,
                                          uint8_t type,
                                          uint8_t value_0,
                                          uint8_t can_bus,
                                          uint8_t node_id,
                                          uint8_t value_3,
                                          uint32_t tick_ms,
                                          uint8_t *output,
                                          size_t output_size)
{
    uint8_t payload[UPPER_MOTOR_EVENT_PAYLOAD_SIZE];

    if (link == NULL)
    {
        return 0U;
    }
    payload[0] = value_0;
    payload[1] = can_bus;
    payload[2] = node_id;
    payload[3] = value_3;
    UpperPcLink_WriteU32(&payload[4], tick_ms);
    return PcProtocol_Encode(type,
                             link->tx_sequence++,
                             payload,
                             sizeof(payload),
                             output,
                             output_size);
}

/* 功能：构造单台 DJI 电机诊断遥测帧；用途：上报反馈、零点、相对位置和各 CAN 接收计数；返回值表示帧长度。 */
size_t UpperPcLink_BuildDjiTelemetry(
                              upper_pc_link_t *link,
                              const upper_pc_dji_telemetry_t *diagnostic,
                              const uint32_t fdcan_rx_count[UPPER_PC_FDCAN_COUNT],
                              uint8_t *output,
                              size_t output_size)
{
    uint8_t payload[UPPER_DJI_TELEMETRY_PAYLOAD_SIZE];
    size_t index;

    if ((link == NULL) || (diagnostic == NULL))
    {
        return 0U;
    }

    (void)memset(payload, 0, sizeof(payload));
    payload[0] = (uint8_t)diagnostic->model;
    payload[1] = diagnostic->can_bus;
    payload[2] = diagnostic->node_id;
    payload[3] = (diagnostic->feedback_received ?
                  UPPER_DJI_FLAG_FEEDBACK_RECEIVED : 0U) |
                 (diagnostic->zero_valid ?
                  UPPER_DJI_FLAG_ZERO_VALID : 0U) |
                 (diagnostic->feedback_fresh ?
                  UPPER_DJI_FLAG_FEEDBACK_FRESH : 0U);
    UpperPcLink_WriteFloat(&payload[4], diagnostic->rotor_position_rad);
    UpperPcLink_WriteFloat(&payload[8],
                           diagnostic->zero_rotor_position_rad);
    UpperPcLink_WriteFloat(&payload[12],
                           diagnostic->relative_output_position_rad);
    if (fdcan_rx_count != NULL)
    {
        for (index = 0U; index < UPPER_PC_FDCAN_COUNT; index++)
        {
            UpperPcLink_WriteU32(
                &payload[UPPER_STATE_DJI_DIAGNOSTIC_SIZE +
                         index * sizeof(uint32_t)],
                fdcan_rx_count[index]);
        }
    }

    return PcProtocol_Encode(PC_MSG_DJI_TELEMETRY,
                             link->tx_sequence++,
                             payload,
                             sizeof(payload),
                             output,
                             output_size);
}

/* 功能：构造电机维护动作结果帧；用途：向上位机反馈动作、总线、节点和执行状态；返回值表示帧长度。 */
size_t UpperPcLink_BuildMotorActionResult(upper_pc_link_t *link,
                                          uint8_t action,
                                          uint8_t can_bus,
                                          uint8_t node_id,
                                          uint8_t status,
                                          uint32_t tick_ms,
                                          uint8_t *output,
                                          size_t output_size)
{
    return UpperPcLink_BuildMotorEvent(link,
                                       PC_MSG_MOTOR_ACTION_RESULT,
                                       action,
                                       can_bus,
                                       node_id,
                                       status,
                                       tick_ms,
                                       output,
                                       output_size);
}

/* 功能：构造电机故障事件帧；用途：向上位机上报协议、总线、节点和故障码；返回值表示帧长度。 */
size_t UpperPcLink_BuildMotorFault(upper_pc_link_t *link,
                                   uint8_t model,
                                   uint8_t can_bus,
                                   uint8_t node_id,
                                   uint8_t error_code,
                                   uint32_t tick_ms,
                                   uint8_t *output,
                                   size_t output_size)
{
    return UpperPcLink_BuildMotorEvent(link,
                                       PC_MSG_FAULT,
                                       model,
                                       can_bus,
                                       node_id,
                                       error_code,
                                       tick_ms,
                                       output,
                                       output_size);
}
