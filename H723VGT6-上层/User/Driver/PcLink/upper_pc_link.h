/**
 * @file upper_pc_link.h
 * @brief 定义上层 PC 链路的协议状态、回调和组帧接口。
 */

#ifndef UPPER_PC_LINK_H
/** 防止 upper_pc_link.h 被重复包含。 */
#define UPPER_PC_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "pc_protocol.h"

/** 基础上层控制命令载荷占用的字节数。 */
#define UPPER_PC_CMD_PAYLOAD_SIZE 34U
/** 兼容版位置控制命令载荷占用的字节数。 */
#define UPPER_PC_POSITION_CMD_PAYLOAD_SIZE 34U
/** 携带 J4310 转矩限制的位置命令载荷占用的字节数。 */
#define UPPER_PC_POSITION_TORQUE_CMD_PAYLOAD_SIZE 42U
/** 携带机构目标和 PID 参数的扩展位置命令载荷占用的字节数。 */
#define UPPER_PC_EXTENDED_POSITION_CMD_PAYLOAD_SIZE 122U
/** 握手请求载荷占用的字节数。 */
#define UPPER_PC_HANDSHAKE_PAYLOAD_SIZE 4U
/** 不带附加配置的电机动作命令载荷占用的字节数。 */
#define UPPER_PC_MOTOR_ACTION_PAYLOAD_SIZE 3U
/** 带一个配置值的电机动作命令载荷占用的字节数。 */
#define UPPER_PC_MOTOR_CONFIG_ACTION_PAYLOAD_SIZE 4U
/** 外部 Flash 信息响应载荷占用的字节数。 */
#define UPPER_PC_FLASH_INFO_PAYLOAD_SIZE 20U
/** 辅助输出控制命令载荷占用的字节数。 */
#define UPPER_PC_AUX_CONTROL_PAYLOAD_SIZE 2U
/** 单帧上报的 DJI 电机诊断条目数量。 */
#define UPPER_PC_DJI_DIAGNOSTIC_COUNT 4U
/** 上位机诊断数据中统计的 FDCAN 控制器数量。 */
#define UPPER_PC_FDCAN_COUNT 3U
/** 上位机机械臂命令中包含的 M3508 电机数量。 */
#define UPPER_PC_ARM_M3508_COUNT 2U

/** 上位机请求 J4310 保存当前位置为零点时使用的动作编号。 */
#define UPPER_PC_ACTION_J4310_SAVE_ZERO 1U
/** 上位机启动或取消 J4310 自动回位时使用的动作编号。 */
#define UPPER_PC_ACTION_J4310_AUTO_RETURN 2U
/** 上位机请求使能 J4310 时使用的动作编号。 */
#define UPPER_PC_ACTION_J4310_ENABLE 3U

typedef void (*upper_pc_aux_control_handler_t)(uint8_t output_bits /* 需要写入辅助控制板的输出位 */,
                                                uint8_t update_mask /* 指定本次允许改变哪些辅助输出位的掩码 */,
                                                void *user_data /* 调用回调函数时传递的用户上下文 */);

/** 保存 上位机链路 初始化和控制所需的配置参数。 */
typedef struct
{
    float kp; /**< 比例增益。 */
    float ki; /**< 积分增益。 */
    float kd; /**< 微分增益。 */
    float integral_limit; /**< 积分累计值的绝对值上限。 */
    float output_limit; /**< 控制器输出绝对值上限。 */
} upper_pc_pid_cfg_t;

/** 保存上位机下发的机械臂控制目标。 */
typedef struct
{
    bool enabled; /**< 对应控制功能是否启用。 */
    bool j4310_commanded; /**< 本周期是否需要向机械臂 J4310 下发控制命令。 */
    bool m3508_enabled; /**< 机械臂两台 M3508 是否允许输出。 */
    bool position_mode; /**< 目标是否采用位置控制模式。 */
    float grip_pos_rad; /**< 上位机链路的目标位置，单位：弧度。 */
    float grip_vel_rad_s; /**< 上位机链路的目标速度，单位：弧度每秒。 */
    float grip_kp; /**< 夹持关节 MIT 位置项的比例增益。 */
    float grip_kd; /**< 夹持关节 MIT 速度项的微分增益。 */
    float grip_torque_nm; /**< 夹持关节 MIT 命令要求的前馈转矩，单位：牛米。 */
    float grip_torque_limit_nm; /**< 上位机链路的转矩上限，单位：牛米。 */
    float m3508_vel_rad_s[UPPER_PC_ARM_M3508_COUNT]; /**< 上位机链路的目标速度，单位：弧度每秒。 */
    float m3508_pos_rad[UPPER_PC_ARM_M3508_COUNT]; /**< 上位机链路的目标位置，单位：弧度。 */
    bool pid_update; /**< 本次命令是否同时更新 PID 参数。 */
    upper_pc_pid_cfg_t m3508_speed_pid; /**< 上位机链路对应控制环的 PID 参数或运行状态。 */
    upper_pc_pid_cfg_t m3508_position_pid; /**< 上位机链路对应控制环的 PID 参数或运行状态。 */
} upper_pc_arm_target_t;

/** 保存上位机下发的单个 M2006 机构控制目标。 */
typedef struct
{
    bool enabled; /**< 对应控制功能是否启用。 */
    bool position_mode; /**< 目标是否采用位置控制模式。 */
    float m2006_vel_rad_s; /**< M2006的目标速度，单位：弧度每秒。 */
    float m2006_pos_rad; /**< M2006的目标位置，单位：弧度。 */
    bool pid_update; /**< 本次命令是否同时更新 PID 参数。 */
    upper_pc_pid_cfg_t m2006_speed_pid; /**< M2006对应控制环的 PID 参数或运行状态。 */
    upper_pc_pid_cfg_t m2006_position_pid; /**< M2006对应控制环的 PID 参数或运行状态。 */
} upper_pc_m2006_target_t;

/** 汇总上位机下发的机械臂、挡板和夹爪控制目标。 */
typedef struct
{
    bool position_mode; /**< 目标是否采用位置控制模式。 */
    upper_pc_arm_target_t arm; /**< 机械臂机构的当前控制目标。 */
    upper_pc_m2006_target_t gate; /**< 挡板机构的当前控制目标。 */
    upper_pc_m2006_target_t gripper; /**< 夹爪机构的当前控制目标。 */
} upper_pc_target_t;

/** 保存 J4310 通信和运行诊断数据。 */
typedef struct
{
    uint32_t accepted_frames; /**< 累计通过全部格式和标识校验的数据帧数量。 */
    uint32_t rejected_format_frames; /**< 因帧格式不合法而拒绝的数据帧数量。 */
    uint32_t rejected_master_id_frames; /**< 因主控标识不匹配而拒绝的数据帧数量。 */
    uint32_t rejected_feedback_id_frames; /**< 因反馈节点编号不匹配而拒绝的数据帧数量。 */
    uint16_t last_can_id; /**< 最近一次处理的数据帧 CAN 标识符。 */
    uint8_t last_dlc; /**< 最近一次处理的数据帧长度。 */
    uint8_t last_data0; /**< 最近一次处理的数据帧首字节。 */
    uint8_t last_result; /**< 最近一次数据帧解析结果。 */
} upper_pc_j4310_rx_diagnostic_t;

/** 保存 J4310 通信和运行诊断数据。 */
typedef struct
{
    uint32_t attempted_frames; /**< 累计尝试发送的数据帧数量。 */
    uint32_t queued_frames; /**< 累计成功进入发送队列的数据帧数量。 */
    uint32_t failed_frames; /**< 累计发送失败的数据帧数量。 */
    uint32_t enable_frames; /**< 累计发送的使能命令帧数量。 */
    uint32_t mit_frames; /**< 累计发送的 MIT 控制帧数量。 */
    uint32_t disable_frames; /**< 累计发送的失能命令帧数量。 */
    uint16_t last_can_id; /**< 最近一次处理的数据帧 CAN 标识符。 */
    uint8_t last_dlc; /**< 最近一次处理的数据帧长度。 */
    uint8_t last_data7; /**< 最近一次发送的数据帧末字节。 */
    bool enable_confirmed; /**< 是否已从 J4310 反馈确认电机进入使能状态。 */
    uint8_t feedback_state; /**< J4310 最近反馈的协议状态码。 */
} upper_pc_j4310_tx_diagnostic_t;

/** 保存 J4310 运行过程中需要集中管理的数据。 */
typedef struct
{
    bool available; /**< 当前拓扑是否配置了可执行自动回位的 J4310。 */
    bool enabled; /**< 对应控制功能是否启用。 */
    bool active; /**< 对应流程当前是否正在运行。 */
    uint8_t stage; /**< 自动回位或动作状态机当前执行的阶段。 */
} upper_j4310_auto_return_status_t;

/** 保存 上位机链路 当前运行状态和中间计算数据。 */
typedef struct
{
    uint8_t robot_state; /**< 上报给上位机的整机运行状态。 */
    uint32_t motor_sent_count; /**< 累计成功下发的电机命令数量。 */
    uint32_t motor_send_fail_count; /**< 累计下发失败的电机命令数量。 */
    uint32_t motor_protocol_block_count; /**< 累计因电机协议未就绪而阻止下发的命令数量。 */
    bool j4310_position_valid; /**< 上报的 J4310 关节角是否来自有效反馈。 */
    float j4310_position_rad; /**< 上位机链路的当前位置，单位：弧度。 */
    uint32_t j4310_bus_rx_frames; /**< J4310 所在 CAN 总线累计接收的帧数。 */
    bool j4310_rx_valid; /**< J4310 接收诊断快照是否有效。 */
    upper_pc_j4310_rx_diagnostic_t j4310_rx; /**< J4310 接收路径的诊断快照。 */
    bool j4310_tx_valid; /**< J4310 发送诊断快照是否有效。 */
    upper_pc_j4310_tx_diagnostic_t j4310_tx; /**< J4310 发送路径的诊断快照。 */
    upper_j4310_auto_return_status_t j4310_auto_return; /**< J4310 自动回位功能的当前状态。 */
} upper_pc_state_t;

/** 保存 上位机链路 运行过程中需要集中管理的数据。 */
typedef struct
{
    uint8_t model; /**< 电机型号。 */
    uint8_t can_bus; /**< 电机或数据帧所在的 CAN 总线编号。 */
    uint8_t node_id; /**< 电机协议节点编号。 */
    bool feedback_received; /**< 是否至少收到过一帧有效反馈。 */
    bool zero_valid; /**< 软件零点是否已经建立。 */
    bool feedback_fresh; /**< 最近反馈是否仍在允许的超时时间内。 */
    float rotor_position_rad; /**< 上位机链路的当前位置，单位：弧度。 */
    float zero_rotor_position_rad; /**< 上位机链路的零点位置，单位：弧度。 */
    float relative_output_position_rad; /**< 上位机链路的当前位置，单位：弧度。 */
} upper_pc_dji_telemetry_t;

typedef void (*upper_pc_cmd_handler_t)(const upper_pc_target_t *target /* 本次需要应用的控制目标 */,
                                       void *user_data /* 调用回调函数时传递的用户上下文 */);
typedef void (*upper_pc_estop_handler_t)(void *user_data /* 调用回调函数时传递的用户上下文 */);
typedef void (*upper_pc_motor_action_handler_t)(uint8_t action /* 需要执行的电机或遥控动作 */,
                                                 uint8_t can_bus /* CAN 总线编号 */,
                                                 uint8_t node_id /* 电机协议节点编号 */,
                                                uint8_t value /* 需要检查、限幅或编码的输入值 */,
                                                void *user_data /* 调用回调函数时传递的用户上下文 */);
/** 保存 上位机链路 运行过程中需要集中管理的数据。 */
typedef struct
{
    pc_parser_t parser; /**< 负责从字节流中恢复完整上位机协议帧的解析器。 */
    upper_pc_cmd_handler_t cmd_handler; /**< 处理对应事件时调用的回调函数。 */
    upper_pc_estop_handler_t estop_handler; /**< 处理对应事件时调用的回调函数。 */
    upper_pc_motor_action_handler_t motor_action_handler; /**< 处理对应事件时调用的回调函数。 */
    upper_pc_aux_control_handler_t aux_control_handler; /**< 处理对应事件时调用的回调函数。 */
    void *user_data; /**< 调用回调函数时原样传回的用户上下文。 */
    volatile uint32_t last_rx_tick_ms; /**< 最近一次收到有效上位机数据的系统毫秒时刻。 */
    volatile bool remote_active; /**< 上位机遥控输入当前是否处于活动状态。 */
    volatile bool session_active; /**< 上位机握手会话当前是否有效。 */
    volatile bool remote_timeout_pending; /**< 是否有尚未被控制层消费的上位机遥控超时事件。 */
    volatile bool handshake_pending; /**< 是否有握手确认等待发送。 */
    volatile uint16_t handshake_sequence; /**< 待确认握手请求的帧序号。 */
    volatile uint32_t handshake_received_tick_ms; /**< 当前已接收的数据字节数。 */
    volatile uint32_t handshake_received_count; /**< 累计接受的有效握手请求数量。 */
    volatile bool flash_info_pending; /**< 是否有 Flash 信息响应等待发送。 */
    volatile uint16_t flash_info_sequence; /**< Flash 信息请求的帧序号。 */
    uint32_t current_rx_tick_ms; /**< 上位机链路的反馈电流的协议原始值。 */
    uint16_t last_rx_sequence; /**< 最近一帧有效上位机消息的序号。 */
    uint16_t tx_sequence; /**< 下一帧主动上报消息使用的发送序号。 */
    uint32_t command_error_count; /**< 累计拒绝的上位机控制命令数量。 */
} upper_pc_link_t;

/* 功能：初始化上位机链路和业务回调；用途：建立协议解析与会话状态；无返回值表示 link 被重置并绑定处理器。 */
void UpperPcLink_Init(upper_pc_link_t *link /* 需要操作的通信链路对象 */,
                      upper_pc_cmd_handler_t cmd_handler /* 对应事件发生时调用的回调函数 */,
                      upper_pc_estop_handler_t estop_handler /* 对应事件发生时调用的回调函数 */,
                      upper_pc_motor_action_handler_t motor_action_handler /* 对应事件发生时调用的回调函数 */,
                      void *user_data /* 调用回调函数时传递的用户上下文 */);
/* 注册辅助控制命令回调；未注册时命令不会触发业务动作。 */
void UpperPcLink_SetAuxControlHandler(upper_pc_link_t *link /* 需要操作的通信链路对象 */,
                                      upper_pc_aux_control_handler_t handler /* 收到有效数据后调用的处理函数 */);
/* 功能：向链路输入一段接收数据；用途：更新时间并驱动流式协议解析；完整消息会自动进入帧处理函数。 */
void UpperPcLink_Push(upper_pc_link_t *link /* 需要操作的通信链路对象 */,
                      const uint8_t *data /* 待处理数据的首地址 */,
                      size_t size /* 待处理数据的字节数 */,
                      uint32_t tick_ms /* 当前系统毫秒时刻 */);
/* 功能：检查并消费遥控链路超时事件；用途：通知控制层停止正在运行的机器人；返回 true 表示本次检测到超时。 */
bool UpperPcLink_IsTimedOut(upper_pc_link_t *link /* 需要操作的通信链路对象 */, uint32_t tick_ms /* 当前系统毫秒时刻 */);
/* 功能：检查握手会话当前是否有效；用途：拒绝未握手或已超时的控制命令；返回 true 表示会话仍在有效期内。 */
bool UpperPcLink_IsSessionActive(upper_pc_link_t *link /* 需要操作的通信链路对象 */, uint32_t tick_ms /* 当前系统毫秒时刻 */);
/* 功能：查询是否有握手确认待发送；用途：驱动确认帧发送流程；返回 true 表示存在待确认握手。 */
bool UpperPcLink_HasHandshakePending(const upper_pc_link_t *link /* 需要操作的通信链路对象 */);
/* 功能：判断握手确认是否已达到保护延时；用途：控制 ACK 的最早发送时刻；返回 true 表示现在可以发送。 */
bool UpperPcLink_IsHandshakeAckDue(const upper_pc_link_t *link /* 需要操作的通信链路对象 */,
                                   uint32_t tick_ms /* 当前系统毫秒时刻 */,
                                   uint32_t guard_ms /* 握手请求到允许发送确认帧之间的保护时间，单位：毫秒 */);
/* 功能：取得待确认握手的序号；用途：匹配 ACK 与请求；返回值为序号，空链路时返回 0。 */
uint16_t UpperPcLink_GetHandshakeSequence(const upper_pc_link_t *link /* 需要操作的通信链路对象 */);
/* 功能：构造握手确认协议帧；用途：向上位机确认 H723 会话；返回 0 表示无待确认握手或构帧失败。 */
size_t UpperPcLink_BuildHandshakeAck(const upper_pc_link_t *link /* 需要操作的通信链路对象 */,
                                     uint8_t *output /* 用于写出计算结果或编码数据的缓冲区 */,
                                     size_t output_size /* 输出缓冲区可用的字节数 */);
/* 功能：标记指定握手 ACK 已发送；用途：结束等待并激活会话；仅序号匹配时状态才改变。 */
void UpperPcLink_MarkHandshakeAckSent(upper_pc_link_t *link /* 需要操作的通信链路对象 */,
                                      uint16_t sequence /* 用于匹配请求和响应的消息序号 */);
/* 功能：检查是否有待回传的 Flash 信息；用途：决定通信任务是否需要发送查询结果；返回 true 表示存在待发信息。 */
bool UpperPcLink_HasFlashInfoPending(const upper_pc_link_t *link /* 需要操作的通信链路对象 */);
/* 功能：读取 Flash 信息请求的序列号；用途：让响应帧与请求正确对应；返回值表示待响应序列号。 */
uint16_t UpperPcLink_GetFlashInfoSequence(const upper_pc_link_t *link /* 需要操作的通信链路对象 */);
/* 功能：构造外部 Flash 信息响应帧；用途：向 PC 回传初始化状态、容量和器件标识；返回值表示完整帧长度。 */
size_t UpperPcLink_BuildFlashInfo(const upper_pc_link_t *link /* 需要操作的通信链路对象 */,
                                  uint8_t init_status /* Flash 初始化函数返回的状态码 */,
                                  bool initialized /* Flash 是否已经成功完成初始化 */,
                                  uint32_t jedec_id /* Flash 的 JEDEC 器件标识 */,
                                  uint32_t capacity_kb /* Flash 总容量，单位：KB */,
                                  uint32_t sector_count /* Flash 扇区总数或本次连续操作的扇区数量 */,
                                  uint16_t page_size_byte /* Flash 单页可编程的字节数 */,
                                  uint32_t sector_size_byte /* Flash 单个扇区的字节数 */,
                                  uint8_t *output /* 用于写出计算结果或编码数据的缓冲区 */,
                                  size_t output_size /* 输出缓冲区可用的字节数 */);
/* 功能：确认指定 Flash 信息响应已经发送；用途：清除一次性待发送标志；无返回值表示链路状态已更新。 */
void UpperPcLink_MarkFlashInfoSent(upper_pc_link_t *link /* 需要操作的通信链路对象 */,
                                   uint16_t sequence /* 用于匹配请求和响应的消息序号 */);
/* 功能：构造机器人基础状态帧；用途：上报状态、链路、发送统计和 J4310 位置；返回值表示完整帧长度。 */
size_t UpperPcLink_BuildState(upper_pc_link_t *link /* 需要操作的通信链路对象 */,
                              const upper_pc_state_t *state /* 需要检查或上报的当前状态 */,
                              uint32_t tick_ms /* 当前系统毫秒时刻 */,
                              uint8_t *output /* 用于写出计算结果或编码数据的缓冲区 */,
                              size_t output_size /* 输出缓冲区可用的字节数 */);
/* 功能：构造单台 DJI 电机诊断遥测帧；用途：上报反馈、零点、相对位置和各 CAN 接收计数；返回值表示帧长度。 */
size_t UpperPcLink_BuildDjiTelemetry(
                              upper_pc_link_t *link /* 需要操作的通信链路对象 */,
                              const upper_pc_dji_telemetry_t *diagnostic /* 用于写出诊断统计的对象 */,
                              const uint32_t fdcan_rx_count[UPPER_PC_FDCAN_COUNT] /* 三个 FDCAN 控制器各自累计接收的帧数数组 */,
                              uint8_t *output /* 用于写出计算结果或编码数据的缓冲区 */,
                              size_t output_size /* 输出缓冲区可用的字节数 */);
/* 功能：构造电机维护动作结果帧；用途：向上位机反馈动作、总线、节点和执行状态；返回值表示帧长度。 */
size_t UpperPcLink_BuildMotorActionResult(upper_pc_link_t *link /* 需要操作的通信链路对象 */,
                                          uint8_t action /* 需要执行的电机或遥控动作 */,
                                          uint8_t can_bus /* CAN 总线编号 */,
                                          uint8_t node_id /* 电机协议节点编号 */,
                                          uint8_t status /* 需要返回给调试端的命令执行状态 */,
                                          uint32_t tick_ms /* 当前系统毫秒时刻 */,
                                          uint8_t *output /* 用于写出计算结果或编码数据的缓冲区 */,
                                          size_t output_size /* 输出缓冲区可用的字节数 */);
/* 功能：构造电机故障事件帧；用途：向上位机上报协议、总线、节点和故障码；返回值表示帧长度。 */
size_t UpperPcLink_BuildMotorFault(upper_pc_link_t *link /* 需要操作的通信链路对象 */,
                                   uint8_t model /* 需要匹配或上报的电机型号 */,
                                   uint8_t can_bus /* CAN 总线编号 */,
                                   uint8_t node_id /* 电机协议节点编号 */,
                                   uint8_t error_code /* 需要记录或上报的电机错误码 */,
                                   uint32_t tick_ms /* 当前系统毫秒时刻 */,
                                   uint8_t *output /* 用于写出计算结果或编码数据的缓冲区 */,
                                   size_t output_size /* 输出缓冲区可用的字节数 */);

#endif
