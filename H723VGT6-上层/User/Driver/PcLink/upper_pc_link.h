/**
 * @file upper_pc_link.h
 * @brief 定义上层 PC 链路的协议状态、回调和组帧接口。
 */

#ifndef UPPER_PC_LINK_H
#define UPPER_PC_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "pc_protocol.h"

#define UPPER_PC_CMD_PAYLOAD_SIZE 34U
#define UPPER_PC_POSITION_CMD_PAYLOAD_SIZE 34U
#define UPPER_PC_POSITION_TORQUE_CMD_PAYLOAD_SIZE 42U
#define UPPER_PC_EXTENDED_POSITION_CMD_PAYLOAD_SIZE 122U
#define UPPER_PC_HANDSHAKE_PAYLOAD_SIZE 4U
#define UPPER_PC_MOTOR_ACTION_PAYLOAD_SIZE 3U
#define UPPER_PC_MOTOR_CONFIG_ACTION_PAYLOAD_SIZE 4U
#define UPPER_PC_FLASH_INFO_PAYLOAD_SIZE 20U
#define UPPER_PC_AUX_CONTROL_PAYLOAD_SIZE 2U
#define UPPER_PC_DJI_DIAGNOSTIC_COUNT 4U
#define UPPER_PC_FDCAN_COUNT 3U
#define UPPER_PC_ARM_M3508_COUNT 2U

#define UPPER_PC_ACTION_J4310_SAVE_ZERO 1U
#define UPPER_PC_ACTION_J4310_AUTO_RETURN 2U
#define UPPER_PC_ACTION_J4310_ENABLE 3U

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
} upper_pc_pid_cfg_t;

typedef struct
{
    bool enabled;
    bool j4310_commanded;
    bool m3508_enabled;
    bool position_mode;
    float grip_pos_rad;
    float grip_vel_rad_s;
    float grip_kp;
    float grip_kd;
    float grip_torque_nm;
    float grip_torque_limit_nm;
    float m3508_vel_rad_s[UPPER_PC_ARM_M3508_COUNT];
    float m3508_pos_rad[UPPER_PC_ARM_M3508_COUNT];
    bool pid_update;
    upper_pc_pid_cfg_t m3508_speed_pid;
    upper_pc_pid_cfg_t m3508_position_pid;
} upper_pc_arm_target_t;

typedef struct
{
    bool enabled;
    bool position_mode;
    float m2006_vel_rad_s;
    float m2006_pos_rad;
    bool pid_update;
    upper_pc_pid_cfg_t m2006_speed_pid;
    upper_pc_pid_cfg_t m2006_position_pid;
} upper_pc_m2006_target_t;

typedef struct
{
    bool position_mode;
    upper_pc_arm_target_t arm;
    upper_pc_m2006_target_t gate;
    upper_pc_m2006_target_t gripper;
} upper_pc_target_t;

typedef struct
{
    uint32_t accepted_frames;
    uint32_t rejected_format_frames;
    uint32_t rejected_master_id_frames;
    uint32_t rejected_feedback_id_frames;
    uint16_t last_can_id;
    uint8_t last_dlc;
    uint8_t last_data0;
    uint8_t last_result;
} upper_pc_j4310_rx_diagnostic_t;

typedef struct
{
    uint32_t attempted_frames;
    uint32_t queued_frames;
    uint32_t failed_frames;
    uint32_t enable_frames;
    uint32_t mit_frames;
    uint32_t disable_frames;
    uint16_t last_can_id;
    uint8_t last_dlc;
    uint8_t last_data7;
    bool enable_confirmed;
    uint8_t feedback_state;
} upper_pc_j4310_tx_diagnostic_t;

typedef struct
{
    bool available;
    bool enabled;
    bool active;
    uint8_t stage;
} upper_j4310_auto_return_status_t;

typedef struct
{
    uint8_t robot_state;
    uint32_t motor_sent_count;
    uint32_t motor_send_fail_count;
    uint32_t motor_protocol_block_count;
    bool j4310_position_valid;
    float j4310_position_rad;
    uint32_t j4310_bus_rx_frames;
    bool j4310_rx_valid;
    upper_pc_j4310_rx_diagnostic_t j4310_rx;
    bool j4310_tx_valid;
    upper_pc_j4310_tx_diagnostic_t j4310_tx;
    upper_j4310_auto_return_status_t j4310_auto_return;
} upper_pc_state_t;

typedef struct
{
    uint8_t model;
    uint8_t can_bus;
    uint8_t node_id;
    bool feedback_received;
    bool zero_valid;
    bool feedback_fresh;
    float rotor_position_rad;
    float zero_rotor_position_rad;
    float relative_output_position_rad;
} upper_pc_dji_telemetry_t;

typedef void (*upper_pc_cmd_handler_t)(const upper_pc_target_t *target,
                                       void *user_data);
typedef void (*upper_pc_estop_handler_t)(void *user_data);
typedef void (*upper_pc_motor_action_handler_t)(uint8_t action,
                                                 uint8_t can_bus,
                                                 uint8_t node_id,
                                                uint8_t value,
                                                void *user_data);
typedef void (*upper_pc_aux_control_handler_t)(uint8_t output_bits,
                                                void *user_data);

typedef struct
{
    pc_parser_t parser;
    upper_pc_cmd_handler_t cmd_handler;
    upper_pc_estop_handler_t estop_handler;
    upper_pc_motor_action_handler_t motor_action_handler;
    upper_pc_aux_control_handler_t aux_control_handler;
    void *user_data;
    volatile uint32_t last_rx_tick_ms;
    volatile bool remote_active;
    volatile bool session_active;
    volatile bool remote_timeout_pending;
    volatile bool handshake_pending;
    volatile uint16_t handshake_sequence;
    volatile uint32_t handshake_received_tick_ms;
    volatile uint32_t handshake_received_count;
    volatile bool flash_info_pending;
    volatile uint16_t flash_info_sequence;
    uint32_t current_rx_tick_ms;
    uint16_t last_rx_sequence;
    uint16_t tx_sequence;
    uint32_t command_error_count;
} upper_pc_link_t;

/* 功能：初始化上位机链路和业务回调；用途：建立协议解析与会话状态；无返回值表示 link 被重置并绑定处理器。 */
void UpperPcLink_Init(upper_pc_link_t *link,
                      upper_pc_cmd_handler_t cmd_handler,
                      upper_pc_estop_handler_t estop_handler,
                      upper_pc_motor_action_handler_t motor_action_handler,
                      void *user_data);
/* 功能：注册气缸与电子急停控制回调；用途：把独立辅助输出命令交给 SPI3 转发层。 */
void UpperPcLink_SetAuxControlHandler(upper_pc_link_t *link,
                                      upper_pc_aux_control_handler_t handler);
/* 功能：向链路输入一段接收数据；用途：更新时间并驱动流式协议解析；完整消息会自动进入帧处理函数。 */
void UpperPcLink_Push(upper_pc_link_t *link,
                      const uint8_t *data,
                      size_t size,
                      uint32_t tick_ms);
/* 功能：检查并消费遥控链路超时事件；用途：通知控制层停止正在运行的机器人；返回 true 表示本次检测到超时。 */
bool UpperPcLink_IsTimedOut(upper_pc_link_t *link, uint32_t tick_ms);
/* 功能：检查握手会话当前是否有效；用途：拒绝未握手或已超时的控制命令；返回 true 表示会话仍在有效期内。 */
bool UpperPcLink_IsSessionActive(upper_pc_link_t *link, uint32_t tick_ms);
/* 功能：查询是否有握手确认待发送；用途：驱动确认帧发送流程；返回 true 表示存在待确认握手。 */
bool UpperPcLink_HasHandshakePending(const upper_pc_link_t *link);
/* 功能：判断握手确认是否已达到保护延时；用途：控制 ACK 的最早发送时刻；返回 true 表示现在可以发送。 */
bool UpperPcLink_IsHandshakeAckDue(const upper_pc_link_t *link,
                                   uint32_t tick_ms,
                                   uint32_t guard_ms);
/* 功能：取得待确认握手的序号；用途：匹配 ACK 与请求；返回值为序号，空链路时返回 0。 */
uint16_t UpperPcLink_GetHandshakeSequence(const upper_pc_link_t *link);
/* 功能：构造握手确认协议帧；用途：向上位机确认 H723 会话；返回 0 表示无待确认握手或构帧失败。 */
size_t UpperPcLink_BuildHandshakeAck(const upper_pc_link_t *link,
                                     uint8_t *output,
                                     size_t output_size);
/* 功能：标记指定握手 ACK 已发送；用途：结束等待并激活会话；仅序号匹配时状态才改变。 */
void UpperPcLink_MarkHandshakeAckSent(upper_pc_link_t *link,
                                      uint16_t sequence);
/* 功能：检查是否有待回传的 Flash 信息；用途：决定通信任务是否需要发送查询结果；返回 true 表示存在待发信息。 */
bool UpperPcLink_HasFlashInfoPending(const upper_pc_link_t *link);
/* 功能：读取 Flash 信息请求的序列号；用途：让响应帧与请求正确对应；返回值表示待响应序列号。 */
uint16_t UpperPcLink_GetFlashInfoSequence(const upper_pc_link_t *link);
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
                                  size_t output_size);
/* 功能：确认指定 Flash 信息响应已经发送；用途：清除一次性待发送标志；无返回值表示链路状态已更新。 */
void UpperPcLink_MarkFlashInfoSent(upper_pc_link_t *link,
                                   uint16_t sequence);
/* 功能：构造机器人基础状态帧；用途：上报状态、链路、发送统计和 J4310 位置；返回值表示完整帧长度。 */
size_t UpperPcLink_BuildState(upper_pc_link_t *link,
                              const upper_pc_state_t *state,
                              uint32_t tick_ms,
                              uint8_t *output,
                              size_t output_size);
/* 功能：构造单台 DJI 电机诊断遥测帧；用途：上报反馈、零点、相对位置和各 CAN 接收计数；返回值表示帧长度。 */
size_t UpperPcLink_BuildDjiTelemetry(
                              upper_pc_link_t *link,
                              const upper_pc_dji_telemetry_t *diagnostic,
                              const uint32_t fdcan_rx_count[UPPER_PC_FDCAN_COUNT],
                              uint8_t *output,
                              size_t output_size);
/* 功能：构造电机维护动作结果帧；用途：向上位机反馈动作、总线、节点和执行状态；返回值表示帧长度。 */
size_t UpperPcLink_BuildMotorActionResult(upper_pc_link_t *link,
                                          uint8_t action,
                                          uint8_t can_bus,
                                          uint8_t node_id,
                                          uint8_t status,
                                          uint32_t tick_ms,
                                          uint8_t *output,
                                          size_t output_size);
/* 功能：构造电机故障事件帧；用途：向上位机上报协议、总线、节点和故障码；返回值表示帧长度。 */
size_t UpperPcLink_BuildMotorFault(upper_pc_link_t *link,
                                   uint8_t model,
                                   uint8_t can_bus,
                                   uint8_t node_id,
                                   uint8_t error_code,
                                   uint32_t tick_ms,
                                   uint8_t *output,
                                   size_t output_size);

#endif
