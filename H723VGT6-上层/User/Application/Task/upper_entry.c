#include "upper_entry.h"

#include <string.h>

#include "can_id.h"
#include "cmsis_os2.h"
#include "comm_runtime.h"
#include "j4310_auto_return.h"
#include "upper_config.h"
#include "upper_motor_port.h"
#include "upper_pc_link.h"
#include "W25Qxx.h"

#define UPPER_CMD_QUEUE_DEPTH  4U
#define UPPER_STATE_PERIOD_MS  50U
#define UPPER_DJI_TELEMETRY_PERIOD_MS 10U
#define UPPER_TX_BUFFER_SIZE   160U
#define UPPER_HANDSHAKE_ACK_GUARD_MS 20U
#define UPPER_J4310_AUTO_RETURN_CONFIG_MAGIC 0x5A523134UL
#define UPPER_J4310_AUTO_RETURN_CONFIG_VERSION 1U
#define UPPER_J4310_AUTO_RETURN_CONFIG_XOR 0xA5C33C5AUL
#define UPPER_J4310_AUTO_RETURN_STORAGE_WAIT_MS 5000U
#define UPPER_J4310_AUTO_RETURN_KP 5.0f
#define UPPER_J4310_AUTO_RETURN_KD 0.5f

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint8_t enabled;
    uint8_t reserved;
    uint32_t checksum;
} upper_j4310_auto_return_config_t;

typedef char upper_j4310_auto_return_config_size_check[
    (sizeof(upper_j4310_auto_return_config_t) == 12U) ? 1 : -1];

static upper_robot_t upper_robot;
static upper_pc_link_t upper_pc_link;
static osMessageQueueId_t upper_cmd_queue;
static volatile bool upper_estop_pending;
static volatile bool upper_motor_action_pending;
static uint8_t upper_motor_action;
static uint8_t upper_motor_action_can_bus;
static uint8_t upper_motor_action_node_id;
static uint8_t upper_motor_action_value;
static bool upper_motor_action_result_pending;
static uint8_t upper_motor_action_result_action;
static uint8_t upper_motor_action_result_can_bus;
static uint8_t upper_motor_action_result_node_id;
static uint8_t upper_motor_action_result_status;
static uint32_t upper_motor_action_result_tick_ms;
static uint32_t upper_motor_offline_latched_mask;
static uint32_t upper_cmd_drop_count;
static uint32_t upper_state_last_sent_tick_ms;
static uint32_t upper_dji_telemetry_last_sent_tick_ms;
static size_t upper_dji_telemetry_next_index;
static j4310_auto_return_t upper_j4310_auto_return;
static bool upper_j4310_auto_return_storage_checked;
static bool upper_j4310_auto_return_storage_ready;
static bool upper_j4310_auto_return_config_enabled;
volatile uint32_t upper_handshake_ack_sent_count;
volatile uint32_t upper_handshake_ack_busy_count;
volatile uint32_t upper_handshake_ack_fail_count;
volatile uint32_t upper_state_sent_count;
volatile uint32_t upper_state_busy_count;
volatile uint32_t upper_state_fail_count;
volatile uint32_t upper_dji_telemetry_sent_count;
volatile uint32_t upper_dji_telemetry_busy_count;
volatile uint32_t upper_dji_telemetry_fail_count;
__ALIGNED(32) static uint8_t upper_tx_buffer[UPPER_TX_BUFFER_SIZE];

/* 功能：接收上位机目标回调并暂存命令；用途：把通信上下文的数据安全移交给控制周期；无返回值表示设置待处理标志。 */
static void UpperEntry_OnPcCmd(const upper_target_t *target, void *user_data)
{
    (void)user_data;
    if (osMessageQueuePut(upper_cmd_queue, target, 0U, 0U) != osOK)
    {
        upper_cmd_drop_count++;
    }
}

/* 功能：接收上位机急停回调；用途：延迟到控制周期执行急停；无返回值表示设置急停待处理标志。 */
static void UpperEntry_OnPcEStop(void *user_data)
{
    (void)user_data;
    upper_estop_pending = true;
}

/* 功能：处理通信层转交的 UART 数据；用途：仅将选定控制通道数据送入上位机协议；无返回值表示数据已消费或忽略。 */
static void UpperEntry_OnUart(comm_uart_channel_t channel,
                              const uint8_t *data,
                              size_t size,
                              void *user_data)
{
    (void)user_data;
    if (channel == COMM_UART_PC)
    {
        UpperEntry_OnPcData(data, size, CommRuntime_GetTickMs());
    }
}

/* 功能：处理通信层转交的 CAN 帧；用途：把反馈同时送给电机端口和 VOFA 桥；无返回值表示完成分发。 */
static void UpperEntry_OnCan(uint8_t can_bus,
                             const can_frame_t *frame,
                             void *user_data)
{
    (void)user_data;
    UpperEntry_OnCanFrame(can_bus, frame, CommRuntime_GetTickMs());
}

/* 功能：接收上位机电机维护动作；用途：暂存 J4310 保存零点等请求供控制周期执行；无返回值表示记录待处理动作。 */
static void UpperEntry_OnPcMotorAction(uint8_t action,
                                       uint8_t can_bus,
                                       uint8_t node_id,
                                       uint8_t value,
                                       void *user_data)
{
    (void)user_data;
    upper_motor_action = action;
    upper_motor_action_can_bus = can_bus;
    upper_motor_action_node_id = node_id;
    upper_motor_action_value = value;
    upper_motor_action_pending = true;
}

static uint32_t UpperEntry_J4310AutoReturnChecksum(
    const upper_j4310_auto_return_config_t *config)
{
    return config->magic ^ ((uint32_t)config->version << 16U) ^
           ((uint32_t)config->enabled << 8U) ^
           UPPER_J4310_AUTO_RETURN_CONFIG_XOR;
}

static uint32_t UpperEntry_J4310AutoReturnConfigAddress(void)
{
    w25q_handle_t *flash;
    uint32_t capacity_bytes;

    flash = W25Q_PortGetDevice();
    if ((flash == NULL) || !flash->is_initialized ||
        (flash->sector_count == 0U))
    {
        return UINT32_MAX;
    }
    capacity_bytes = flash->capacity_kb * 1024UL;
    if (capacity_bytes < W25Q_SECTOR_SIZE_BYTE)
    {
        return UINT32_MAX;
    }
    return capacity_bytes - W25Q_SECTOR_SIZE_BYTE;
}

static void UpperEntry_LoadJ4310AutoReturnConfig(uint32_t tick_ms)
{
    upper_j4310_auto_return_config_t config;
    w25q_handle_t *flash;
    uint32_t address;
    bool enabled;

    if (upper_j4310_auto_return_storage_checked)
    {
        return;
    }
    flash = W25Q_PortGetDevice();
    if ((flash == NULL) || !flash->is_initialized)
    {
        if (tick_ms >= UPPER_J4310_AUTO_RETURN_STORAGE_WAIT_MS)
        {
            upper_j4310_auto_return_storage_checked = true;
            upper_j4310_auto_return_storage_ready = false;
            J4310AutoReturn_Init(&upper_j4310_auto_return, false);
        }
        return;
    }

    address = UpperEntry_J4310AutoReturnConfigAddress();
    if ((address == UINT32_MAX) ||
        (W25Q_ReadData(flash,
                       address,
                       (uint8_t *)&config,
                       sizeof(config)) != W25Q_OK))
    {
        upper_j4310_auto_return_storage_checked = true;
        upper_j4310_auto_return_storage_ready = false;
        J4310AutoReturn_Init(&upper_j4310_auto_return, false);
        return;
    }

    enabled = (config.magic == UPPER_J4310_AUTO_RETURN_CONFIG_MAGIC) &&
              (config.version ==
               UPPER_J4310_AUTO_RETURN_CONFIG_VERSION) &&
              (config.enabled <= 1U) && (config.reserved == 0U) &&
              (config.checksum ==
               UpperEntry_J4310AutoReturnChecksum(&config)) &&
              (config.enabled != 0U);
    upper_j4310_auto_return_storage_checked = true;
    upper_j4310_auto_return_storage_ready = true;
    upper_j4310_auto_return_config_enabled = enabled;
    J4310AutoReturn_Init(&upper_j4310_auto_return, enabled);
}

static bool UpperEntry_SaveJ4310AutoReturnConfig(bool enabled)
{
    upper_j4310_auto_return_config_t config;
    upper_j4310_auto_return_config_t verify;
    w25q_handle_t *flash;
    uint32_t address;

    if (!upper_j4310_auto_return_storage_ready)
    {
        return false;
    }
    flash = W25Q_PortGetDevice();
    address = UpperEntry_J4310AutoReturnConfigAddress();
    if ((flash == NULL) || (address == UINT32_MAX))
    {
        return false;
    }
    (void)memset(&config, 0, sizeof(config));
    config.magic = UPPER_J4310_AUTO_RETURN_CONFIG_MAGIC;
    config.version = UPPER_J4310_AUTO_RETURN_CONFIG_VERSION;
    config.enabled = enabled ? 1U : 0U;
    config.checksum = UpperEntry_J4310AutoReturnChecksum(&config);
    if ((W25Q_WriteData(flash,
                        address,
                        (const uint8_t *)&config,
                        sizeof(config)) != W25Q_OK) ||
        (W25Q_ReadData(flash,
                       address,
                       (uint8_t *)&verify,
                       sizeof(verify)) != W25Q_OK) ||
        (memcmp(&config, &verify, sizeof(config)) != 0))
    {
        return false;
    }
    return true;
}

static void UpperEntry_ReleaseJ4310AutoReturn(void)
{
    upper_target_t target;

    target = upper_robot.target;
    target.arm.enabled = false;
    target.arm.j4310_commanded = false;
    UpperRobot_SetTarget(&upper_robot, &target);
}

static void UpperEntry_ServiceJ4310AutoReturn(uint32_t tick_ms)
{
    upper_j4310_feedback_t feedback;
    upper_target_t target;
    bool feedback_fresh;
    bool was_active;

    feedback_fresh = UpperMotorPort_GetJ4310Feedback(
                         CAN_BUS_ARM_J4310,
                         NODE_ARM_J4310,
                         &feedback);
    was_active = upper_j4310_auto_return.owns_control;
    J4310AutoReturn_Update(
        &upper_j4310_auto_return,
        tick_ms,
        feedback_fresh,
        feedback_fresh ? feedback.position_rad : 0.0f,
        feedback_fresh ? feedback.velocity_rad_s : 0.0f,
        upper_robot.state != ROBOT_ERROR);
    if (was_active && !upper_j4310_auto_return.owns_control)
    {
        UpperEntry_ReleaseJ4310AutoReturn();
        return;
    }
    if (!upper_j4310_auto_return.owns_control)
    {
        return;
    }

    target = upper_robot.target;
    target.arm.enabled = true;
    target.arm.j4310_commanded = false;
    target.arm.grip_pos_rad =
        upper_j4310_auto_return.target_position_rad;
    target.arm.grip_vel_rad_s =
        upper_j4310_auto_return.target_velocity_rad_s;
    target.arm.grip_kp = UPPER_J4310_AUTO_RETURN_KP;
    target.arm.grip_kd = UPPER_J4310_AUTO_RETURN_KD;
    target.arm.grip_torque_nm = 0.0f;
    UpperRobot_SetTarget(&upper_robot, &target);
    UpperRobot_Start(&upper_robot);
}

/* 功能：初始化上层入口、机器人、链路和通信回调；用途：完成用户应用启动；返回 true 表示所有子模块初始化成功。 */
bool UpperEntry_Init(void)
{
    if (!UpperMotorPort_Init(upper_motor_cfg, UPPER_MOTOR_COUNT))
    {
        return false;
    }
    upper_cmd_queue = osMessageQueueNew(UPPER_CMD_QUEUE_DEPTH,
                                        sizeof(upper_target_t),
                                        NULL);
    if (upper_cmd_queue == NULL)
    {
        return false;
    }

    UpperPcLink_Init(&upper_pc_link,
                     UpperEntry_OnPcCmd,
                     UpperEntry_OnPcEStop,
                     UpperEntry_OnPcMotorAction,
                     NULL);
    J4310AutoReturn_Init(&upper_j4310_auto_return, false);
    upper_j4310_auto_return_storage_checked = false;
    upper_j4310_auto_return_storage_ready = false;
    upper_j4310_auto_return_config_enabled = false;
    CommRuntime_SetHandlers(UpperEntry_OnUart, UpperEntry_OnCan, NULL);
    return UpperRobot_Init(&upper_robot, UpperMotorPort_Send, NULL);
}

/* 功能：处理待执行的上位机整机目标；用途：更新机器人目标并按需启动运行；无返回值表示待处理标志被消费。 */
static void UpperEntry_ProcessCmd(void)
{
    upper_target_t target;
    bool received;
    bool j4310_commanded;

    received = false;
    j4310_commanded = false;
    while (osMessageQueueGet(upper_cmd_queue, &target, NULL, 0U) == osOK)
    {
        received = true;
        j4310_commanded = j4310_commanded ||
                          target.arm.j4310_commanded;
    }

    if (received)
    {
        if (j4310_commanded)
        {
            J4310AutoReturn_Cancel(&upper_j4310_auto_return);
        }
        UpperRobot_SetTarget(&upper_robot, &target);
        UpperRobot_Start(&upper_robot);
    }
}

/* 功能：执行待处理的电机维护动作并发送结果；用途：完成保存零点等异步请求；无返回值表示结果已发送或计数失败。 */
static void UpperEntry_ProcessMotorAction(uint32_t tick_ms)
{
    uint8_t action;
    uint8_t can_bus;
    uint8_t node_id;
    uint8_t value;
    bool success;
    bool feedback_fresh;
    bool was_active;
    upper_j4310_feedback_t feedback;

    if (!upper_motor_action_pending)
    {
        return;
    }
    action = upper_motor_action;
    can_bus = upper_motor_action_can_bus;
    node_id = upper_motor_action_node_id;
    value = upper_motor_action_value;
    upper_motor_action_pending = false;

    success = false;
    if (action == UPPER_PC_ACTION_J4310_SAVE_ZERO)
    {
        J4310AutoReturn_Cancel(&upper_j4310_auto_return);
        UpperRobot_Stop(&upper_robot);
        success = UpperMotorPort_SaveJ4310Zero(can_bus, node_id);
    }
    else if ((action == UPPER_PC_ACTION_J4310_AUTO_RETURN) &&
             (can_bus == CAN_BUS_ARM_J4310) &&
             (node_id == NODE_ARM_J4310) && (value <= 1U))
    {
        was_active = upper_j4310_auto_return.owns_control;
        success = UpperEntry_SaveJ4310AutoReturnConfig(value != 0U);
        if (success)
        {
            feedback_fresh = UpperMotorPort_GetJ4310Feedback(
                                 CAN_BUS_ARM_J4310,
                                 NODE_ARM_J4310,
                                 &feedback);
            upper_j4310_auto_return_config_enabled = value != 0U;
            J4310AutoReturn_Configure(&upper_j4310_auto_return,
                                      value != 0U,
                                      feedback_fresh);
            if (was_active)
            {
                UpperEntry_ReleaseJ4310AutoReturn();
            }
        }
    }
    upper_motor_action_result_status = success ? 0U : 1U;
    upper_motor_action_result_action = action;
    upper_motor_action_result_can_bus = can_bus;
    upper_motor_action_result_node_id = node_id;
    upper_motor_action_result_tick_ms = tick_ms;
    upper_motor_action_result_pending = true;
}

/* 功能：检查电机健康状态并处理新故障；用途：触发安全停机并向上位机上报故障；无返回值表示完成本周期巡检。 */
static void UpperEntry_CheckMotorHealth(uint32_t tick_ms)
{
    upper_motor_health_t health;
    upper_motor_fault_t pending_fault;
    uint32_t new_offline_mask;
    size_t index;

    if (UpperMotorPort_GetPendingFault(&pending_fault) ||
        !UpperMotorPort_GetHealth(tick_ms, &health) ||
        UpperMotorPort_GetPendingFault(&pending_fault))
    {
        return;
    }
    new_offline_mask = health.offline_mask & ~upper_motor_offline_latched_mask;
    upper_motor_offline_latched_mask &= health.offline_mask;
    for (index = 0U; index < UPPER_MOTOR_COUNT; index++)
    {
        uint32_t mask;

        mask = 1UL << index;
        if ((new_offline_mask & mask) != 0U)
        {
            UpperMotorPort_RecordExternalFault(&upper_motor_cfg[index],
                                                UPPER_MOTOR_ERROR_FEEDBACK_TIMEOUT,
                                                tick_ms);
            upper_motor_offline_latched_mask |= mask;
            break;
        }
    }
}

/* 功能：处理上位机链路超时；用途：停止其他机构但保持 J4310 的已确认使能和最后目标；无返回值表示目标已更新。 */
static void UpperEntry_HandlePcTimeout(void)
{
    upper_target_t target;

    if (upper_robot.state != ROBOT_RUN)
    {
        return;
    }
    target = upper_robot.target;
    target.arm.m3508_enabled = false;
    target.conveyor.enabled = false;
    target.gripper.enabled = false;
    UpperRobot_SetTarget(&upper_robot, &target);
}

/* 功能：在保护延时后发送待处理握手确认；用途：建立上位机控制会话；无返回值表示发送状态写入链路和统计。 */
static void UpperEntry_SendHandshakeAck(uint32_t tick_ms)
{
    size_t frame_size;
    uint16_t sequence;

    if (!UpperPcLink_IsHandshakeAckDue(&upper_pc_link,
                                       tick_ms,
                                       UPPER_HANDSHAKE_ACK_GUARD_MS))
    {
        return;
    }

    if (!CommRuntime_PcTxReady())
    {
        upper_handshake_ack_busy_count++;
        return;
    }

    sequence = UpperPcLink_GetHandshakeSequence(&upper_pc_link);
    frame_size = UpperPcLink_BuildHandshakeAck(&upper_pc_link,
                                               upper_tx_buffer,
                                               sizeof(upper_tx_buffer));
    if ((frame_size > 0U) &&
        CommRuntime_PcTransmit(upper_tx_buffer, (uint16_t)frame_size))
    {
        UpperPcLink_MarkHandshakeAckSent(&upper_pc_link, sequence);
        upper_handshake_ack_sent_count++;
    }
    else
    {
        upper_handshake_ack_fail_count++;
    }
}

/* Flash diagnostics are independent from the motor-control session. */
static void UpperEntry_SendFlashInfo(void)
{
    size_t frame_size;
    uint16_t sequence;
    w25q_handle_t *flash_device;

    if (!UpperPcLink_HasFlashInfoPending(&upper_pc_link) ||
        !CommRuntime_PcTxReady())
    {
        return;
    }

    sequence = UpperPcLink_GetFlashInfoSequence(&upper_pc_link);
    flash_device = W25Q_PortGetDevice();
    frame_size = UpperPcLink_BuildFlashInfo(
                     &upper_pc_link,
                     (uint8_t)W25Q_PortGetInitStatus(),
                     flash_device->is_initialized,
                     flash_device->flash_id,
                     flash_device->capacity_kb,
                     flash_device->sector_count,
                     flash_device->page_size_byte,
                     W25Q_SECTOR_SIZE_BYTE,
                     upper_tx_buffer,
                     sizeof(upper_tx_buffer));
    if ((frame_size > 0U) &&
        CommRuntime_PcTransmit(upper_tx_buffer, (uint16_t)frame_size))
    {
        UpperPcLink_MarkFlashInfoSent(&upper_pc_link, sequence);
    }
}

/* 功能：按周期发送机器人状态；用途：向上位机报告运行态、通信统计和关节位置；无返回值表示结果写入发送统计。 */
static void UpperEntry_SendState(uint32_t tick_ms)
{
    size_t frame_size;
    upper_motor_fault_t fault;
    upper_j4310_rx_diagnostic_t j4310_rx_diagnostic;
    upper_j4310_tx_diagnostic_t j4310_tx_diagnostic;
    upper_j4310_auto_return_status_t j4310_auto_return_status;
    float j4310_position_rad;
    bool j4310_position_valid;
    bool j4310_diagnostic_valid;
    bool j4310_tx_diagnostic_valid;

    UpperEntry_SendHandshakeAck(tick_ms);
    if (!UpperPcLink_IsSessionActive(&upper_pc_link, tick_ms))
    {
        UpperEntry_SendFlashInfo();
        return;
    }
    if (upper_motor_action_result_pending && CommRuntime_PcTxReady())
    {
        frame_size = UpperPcLink_BuildMotorActionResult(
                         &upper_pc_link,
                         upper_motor_action_result_action,
                         upper_motor_action_result_can_bus,
                         upper_motor_action_result_node_id,
                         upper_motor_action_result_status,
                         upper_motor_action_result_tick_ms,
                         upper_tx_buffer,
                         sizeof(upper_tx_buffer));
        if ((frame_size > 0U) &&
            CommRuntime_PcTransmit(upper_tx_buffer, (uint16_t)frame_size))
        {
            upper_motor_action_result_pending = false;
        }
    }
    if (UpperMotorPort_GetPendingFault(&fault) && CommRuntime_PcTxReady())
    {
        frame_size = UpperPcLink_BuildMotorFault(
                         &upper_pc_link,
                         (uint8_t)fault.model,
                         fault.can_bus,
                         fault.node_id,
                         fault.error_code,
                         fault.tick_ms,
                         upper_tx_buffer,
                         sizeof(upper_tx_buffer));
        if ((frame_size > 0U) &&
            CommRuntime_PcTransmit(upper_tx_buffer, (uint16_t)frame_size))
        {
            UpperMotorPort_MarkFaultSent(fault.sequence);
        }
    }
    UpperEntry_SendFlashInfo();
    if ((tick_ms - upper_state_last_sent_tick_ms) < UPPER_STATE_PERIOD_MS)
    {
        return;
    }

    if (!CommRuntime_PcTxReady())
    {
        upper_state_busy_count++;
        return;
    }

    j4310_position_valid = UpperMotorPort_GetJ4310OutputPosition(
                               CAN_BUS_ARM_J4310,
                               NODE_ARM_J4310,
                               &j4310_position_rad);
    j4310_diagnostic_valid = UpperMotorPort_GetJ4310RxDiagnostic(
                                 CAN_BUS_ARM_J4310,
                                 NODE_ARM_J4310,
                                 &j4310_rx_diagnostic);
    j4310_tx_diagnostic_valid = UpperMotorPort_GetJ4310TxDiagnostic(
                                    CAN_BUS_ARM_J4310,
                                    NODE_ARM_J4310,
                                    &j4310_tx_diagnostic);
    j4310_auto_return_status.storage_ready =
        upper_j4310_auto_return_storage_ready;
    j4310_auto_return_status.enabled =
        upper_j4310_auto_return_config_enabled;
    j4310_auto_return_status.active =
        upper_j4310_auto_return.owns_control;
    j4310_auto_return_status.stage =
        (uint8_t)upper_j4310_auto_return.stage;
    frame_size = UpperPcLink_BuildState(&upper_pc_link,
                                         &upper_robot,
                                         tick_ms,
                                         j4310_position_valid,
                                         j4310_position_valid ?
                                         j4310_position_rad : 0.0f,
                                         comm_fdcan_rx_count[
                                             CAN_BUS_ARM_J4310 - 1U],
                                         j4310_diagnostic_valid ?
                                         &j4310_rx_diagnostic : NULL,
                                         j4310_tx_diagnostic_valid ?
                                         &j4310_tx_diagnostic : NULL,
                                         &j4310_auto_return_status,
                                         upper_tx_buffer,
                                         sizeof(upper_tx_buffer));
    if ((frame_size > 0U) &&
        CommRuntime_PcTransmit(upper_tx_buffer, (uint16_t)frame_size))
    {
        upper_state_last_sent_tick_ms = tick_ms;
        upper_state_sent_count++;
    }
    else
    {
        upper_state_fail_count++;
    }
}

/* 功能：轮询发送 DJI 电机诊断遥测；用途：分时上报多台电机与 FDCAN 接收计数；无返回值表示推进遥测索引。 */
static void UpperEntry_SendDjiTelemetry(uint32_t tick_ms)
{
    upper_dji_diagnostic_t diagnostics[UPPER_PC_DJI_DIAGNOSTIC_COUNT];
    uint32_t fdcan_rx_counts[UPPER_PC_FDCAN_COUNT];
    size_t diagnostic_count;
    size_t index;
    size_t frame_size;

    if (!UpperPcLink_IsSessionActive(&upper_pc_link, tick_ms) ||
        ((tick_ms - upper_dji_telemetry_last_sent_tick_ms) <
         UPPER_DJI_TELEMETRY_PERIOD_MS))
    {
        return;
    }
    if (!CommRuntime_PcTxReady())
    {
        upper_dji_telemetry_busy_count++;
        return;
    }

    diagnostic_count = UpperMotorPort_GetDjiDiagnostics(
                           tick_ms,
                           diagnostics,
                           UPPER_PC_DJI_DIAGNOSTIC_COUNT);
    if (diagnostic_count == 0U)
    {
        return;
    }
    if (upper_dji_telemetry_next_index >= diagnostic_count)
    {
        upper_dji_telemetry_next_index = 0U;
    }
    for (index = 0U; index < UPPER_PC_FDCAN_COUNT; index++)
    {
        fdcan_rx_counts[index] = comm_fdcan_rx_count[index];
    }
    frame_size = UpperPcLink_BuildDjiTelemetry(
                     &upper_pc_link,
                     &diagnostics[upper_dji_telemetry_next_index],
                     fdcan_rx_counts,
                     upper_tx_buffer,
                     sizeof(upper_tx_buffer));
    if ((frame_size > 0U) &&
        CommRuntime_PcTransmit(upper_tx_buffer, (uint16_t)frame_size))
    {
        upper_dji_telemetry_last_sent_tick_ms = tick_ms;
        upper_dji_telemetry_next_index++;
        upper_dji_telemetry_sent_count++;
    }
    else
    {
        upper_dji_telemetry_fail_count++;
    }
}

/* 功能：执行上层应用的 1 ms 主控制周期；用途：处理命令、控制电机、检查故障并发送状态；无返回值表示完成一次调度。 */
void UpperEntry_Control1ms(uint32_t tick_ms)
{
    UpperMotorPort_BeginCycle(tick_ms);
    UpperEntry_LoadJ4310AutoReturnConfig(tick_ms);
    UpperEntry_ProcessCmd();
    UpperEntry_ProcessMotorAction(tick_ms);
    if (upper_estop_pending)
    {
        upper_estop_pending = false;
        J4310AutoReturn_Cancel(&upper_j4310_auto_return);
        UpperRobot_EStop(&upper_robot);
    }
    else
    {
        if (UpperPcLink_IsTimedOut(&upper_pc_link, tick_ms))
        {
            UpperEntry_HandlePcTimeout();
        }
    }

    UpperEntry_ServiceJ4310AutoReturn(tick_ms);
    UpperRobot_Control1ms(&upper_robot, tick_ms);
    if (!UpperMotorPort_Flush())
    {
        upper_robot.motor_manager.send_fail_count++;
    }
    UpperEntry_CheckMotorHealth(tick_ms);
    UpperEntry_SendState(tick_ms);
    UpperEntry_SendDjiTelemetry(tick_ms);
}

/* 功能：接收上位机原始字节流；用途：作为外部入口推进链路解析；无返回值表示数据已交给 UpperPcLink。 */
void UpperEntry_OnPcData(const uint8_t *data,
                         size_t size,
                         uint32_t tick_ms)
{
    UpperPcLink_Push(&upper_pc_link, data, size, tick_ms);
}

/* 功能：接收外部 CAN 帧入口；用途：把电机反馈交给上层电机端口解析；无返回值表示帧已分发。 */
void UpperEntry_OnCanFrame(uint8_t can_bus,
                           const can_frame_t *frame,
                           uint32_t tick_ms)
{
    UpperMotorPort_OnFrame(can_bus, frame, tick_ms);
}

/* 功能：使用 HAL 毫秒时基调用上层控制周期；用途：提供给系统任务的固定入口；无返回值表示完成本次 1 ms 调用。 */
void App_Control1ms(void)
{
    UpperEntry_Control1ms(CommRuntime_GetTickMs());
}
