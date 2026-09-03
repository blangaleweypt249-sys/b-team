/**
 * @file upper_motor_port.h
 * @brief 声明上层电机端口、反馈诊断和故障上报接口。
 */

#ifndef UPPER_MOTOR_PORT_H
#define UPPER_MOTOR_PORT_H /**< 防止 upper_motor_port.h 被重复包含。 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "can_frame.h"
#include "motor_manager.h"

/** 标识上层电机拓扑中每台机构电机的固定槽位。 */
typedef enum
{
    UPPER_MOTOR_ARM_M3508_1, /**< 第一台机械臂 M3508。 */
    UPPER_MOTOR_ARM_M3508_2, /**< 第二台机械臂 M3508。 */
    UPPER_MOTOR_ARM_J4310, /**< 机械臂 J4310 关节电机。 */
    UPPER_MOTOR_GATE_M2006, /**< 挡板机构 M2006。 */
    UPPER_MOTOR_GRIPPER_M2006, /**< 夹爪机构 M2006。 */
    UPPER_MOTOR_COUNT /**< 上层电机拓扑中的电机总数。 */
} upper_motor_id_t;

/** 保存 电机 运行过程中需要集中管理的数据。 */
typedef struct
{
    uint32_t active_mask; /**< 已配置并参与管理的电机位图。 */
    uint32_t offline_mask; /**< 反馈超时的离线电机位图。 */
    uint32_t fault_mask; /**< 已上报运行故障的电机位图。 */
    uint32_t protocol_block_mask; /**< 因协议冲突而禁止输出的电机位图。 */
} upper_motor_health_t;

/** 保存 电机 运行过程中需要集中管理的数据。 */
typedef struct
{
    motor_model_t model; /**< 电机型号。 */
    uint8_t can_bus; /**< 发生故障的电机所在 CAN 总线编号。 */
    uint8_t node_id; /**< 电机协议节点编号。 */
    uint8_t error_code; /**< 待上报的电机故障码。 */
    uint32_t tick_ms; /**< 记录该事件时的系统毫秒时刻。 */
    uint32_t sequence; /**< 故障记录的递增序号。 */
} upper_motor_fault_t;

/** 保存 模块 通信和运行诊断数据。 */
typedef struct
{
    motor_model_t model; /**< 电机型号。 */
    uint8_t can_bus; /**< 被诊断 DJI 电机所在的 CAN 总线编号。 */
    uint8_t node_id; /**< 电机协议节点编号。 */
    bool feedback_received; /**< 是否至少收到过一帧有效反馈。 */
    bool zero_valid; /**< 软件零点是否已经建立。 */
    bool feedback_fresh; /**< 最近反馈是否仍在允许的超时时间内。 */
    float rotor_position_rad; /**< DJI 电机当前累计转子位置，单位：弧度。 */
    float zero_rotor_position_rad; /**< DJI 电机软件零点对应的累计转子位置，单位：弧度。 */
    float relative_output_position_rad; /**< DJI 电机减速后相对软件零点的输出轴位置，单位：弧度。 */
} upper_dji_diagnostic_t;

/** 保存 J4310 通信和运行诊断数据。 */
typedef struct
{
    uint32_t frames_seen; /**< 累计检查到的候选数据帧数量。 */
    uint32_t accepted_frames; /**< 累计通过全部格式和标识校验的数据帧数量。 */
    uint32_t rejected_format_frames; /**< 因帧格式不合法而拒绝的数据帧数量。 */
    uint32_t rejected_master_id_frames; /**< 因主控标识不匹配而拒绝的数据帧数量。 */
    uint32_t rejected_feedback_id_frames; /**< 因反馈节点编号不匹配而拒绝的数据帧数量。 */
    uint16_t last_can_id; /**< 最近一次发送的 J4310 帧 CAN 标识符。 */
    uint8_t last_dlc; /**< 最近一次发送的 J4310 帧长度。 */
    uint8_t last_data0; /**< 最近一次处理的数据帧首字节。 */
    uint8_t last_result; /**< 最近一次数据帧解析结果。 */
} upper_j4310_rx_diagnostic_t;

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
} upper_j4310_tx_diagnostic_t;

/** 保存 J4310 最近一次有效反馈及其时间信息。 */
typedef struct
{
    float position_rad; /**< J4310的当前位置，单位：弧度。 */
    float velocity_rad_s; /**< J4310的当前速度，单位：弧度每秒。 */
    float torque_nm; /**< J4310的反馈转矩，单位：牛米。 */
    uint32_t updated_at_ms; /**< 最近一次收到有效反馈的系统毫秒时刻。 */
    uint8_t state; /**< J4310 最近反馈的协议状态码。 */
} upper_j4310_feedback_t;

/** 保存 M2006 最近一次有效反馈及其时间信息。 */
typedef struct
{
    float position_rad; /**< M2006的当前位置，单位：弧度。 */
    float velocity_rad_s; /**< M2006的当前速度，单位：弧度每秒。 */
    float current_a; /**< M2006的反馈电流，单位：安培。 */
    uint32_t updated_at_ms; /**< 最近一次收到有效反馈的系统毫秒时刻。 */
} upper_m2006_feedback_t;

#define UPPER_MOTOR_ERROR_FEEDBACK_TIMEOUT 0xF0U /**< 电机反馈超时事件上报给上位机时使用的错误码。 */

/* 功能：初始化电机协议驱动、拓扑状态和故障状态；用途：建立上层统一电机端口；返回 true 表示全部型号初始化成功。 */
bool UpperMotorPort_Init(const motor_cfg_t *cfg /**< 当前电机的型号、总线及节点配置 */, size_t motor_count /**< 调用方提供的电机配置数量 */);
/* 功能：开始新的电机发送周期并清空 DJI 分组暂存；用途：保证各槽位只包含本周期命令；无返回值表示周期状态已复位。 */
void UpperMotorPort_BeginCycle(uint32_t tick_ms /**< 当前系统毫秒时刻 */);
/* 功能：按配置型号路由统一电机命令；用途：作为 MotorManager 的发送回调；返回 true 表示命令被对应协议接受。 */
bool UpperMotorPort_Send(const motor_cfg_t *cfg /**< 当前电机的型号、总线及节点配置 */,
                         const motor_cmd_t *cmd /**< 待按电机型号路由的统一控制命令 */,
                         void *user_data /**< 调用回调函数时传递的用户上下文 */);
/* 功能：发送本周期所有待处理 DJI 分组帧；用途：在各电机目标均暂存后统一输出；返回 true 表示所有组发送成功。 */
bool UpperMotorPort_Flush(void);
/* 功能：按总线和标识符分发电机反馈帧；用途：更新 J4310、M3508 或 M2006 驱动状态；无返回值表示已尝试路由。 */
void UpperMotorPort_OnFrame(uint8_t can_bus /**< CAN 总线编号 */,
                             const can_frame_t *frame /**< 待解析的 CAN 接收帧 */,
                             uint32_t tick_ms /**< 当前系统毫秒时刻 */);
/* 功能：检查全部电机的反馈超时和协议故障；用途：形成整机健康结论并记录新故障；返回 true 表示所有电机健康。 */
bool UpperMotorPort_GetHealth(uint32_t tick_ms /**< 当前系统毫秒时刻 */,
                               upper_motor_health_t *health /**< 用于写出电机在线状态和故障信息 */);
/* 功能：仅发送指定 J4310 的协议使能帧；用途：上电恢复时先使能电机而不下发运动目标；返回 true 表示使能帧发送成功。 */
bool UpperMotorPort_EnableJ4310(uint8_t can_bus /**< CAN 总线编号 */, uint8_t node_id /**< 电机协议节点编号 */);
/* 功能：校验反馈并执行 J4310 保存零点序列；用途：安全完成关节机械零位标定；返回 true 表示命令序列发送成功。 */
bool UpperMotorPort_SaveJ4310Zero(uint8_t can_bus /**< CAN 总线编号 */, uint8_t node_id /**< 电机协议节点编号 */);
/* 功能：读取 J4310 经方向和减速比换算后的输出轴位置；用途：向应用层提供机械关节角；返回 true 表示反馈新鲜有效。 */
bool UpperMotorPort_GetJ4310OutputPosition(uint8_t can_bus /**< CAN 总线编号 */,
                                           uint8_t node_id /**< 电机协议节点编号 */,
                                           float *position_rad /**< 用于写出 J4310 机构位置的地址，单位：弧度 */);
/* 功能：读取 J4310 自动归零所需的位置、速度和状态；用途：仅向上层提供新鲜且无故障的反馈；返回 true 表示可安全用于轨迹控制。 */
bool UpperMotorPort_GetJ4310Feedback(uint8_t can_bus /**< CAN 总线编号 */,
                                     uint8_t node_id /**< 电机协议节点编号 */,
                                     upper_j4310_feedback_t *feedback /**< 用于写出最新 J4310 反馈的对象 */);
/* 读取机构坐标系下的新鲜 M2006 位置、速度和电流。 */
bool UpperMotorPort_GetM2006Feedback(uint8_t can_bus /**< CAN 总线编号 */,
                                     uint8_t node_id /**< 电机协议节点编号 */,
                                     upper_m2006_feedback_t *feedback /**< 用于写出最新 M2006 反馈的对象 */);
/* 功能：读取 J4310 协议接收诊断；用途：区分 FDCAN 有帧但格式、Master ID 或反馈 ID 不匹配；返回 true 表示目标已配置且快照有效。 */
bool UpperMotorPort_GetJ4310RxDiagnostic(
    uint8_t can_bus /**< CAN 总线编号 */,
    uint8_t node_id /**< 电机协议节点编号 */,
    upper_j4310_rx_diagnostic_t *diagnostic /**< 用于写出上层 J4310 接收诊断的对象 */);
/* 功能：读取 J4310 发送与使能确认快照；用途：向上位机区分未发送、TX 入队失败和电机未确认；返回 true 表示节点已配置。 */
bool UpperMotorPort_GetJ4310TxDiagnostic(
    uint8_t can_bus /**< CAN 总线编号 */,
    uint8_t node_id /**< 电机协议节点编号 */,
    upper_j4310_tx_diagnostic_t *diagnostic /**< 用于写出上层 J4310 发送诊断的对象 */);
/* 功能：收集已配置 DJI 电机的诊断快照；用途：上报转子位置、零点、输出位置和反馈新鲜度；返回值表示写入条目数。 */
size_t UpperMotorPort_GetDjiDiagnostics(uint32_t tick_ms /**< 当前系统毫秒时刻 */,
                                        upper_dji_diagnostic_t *diagnostics /**< 用于写出 DJI 电机诊断条目数组 */,
                                        size_t capacity /**< 调用方提供的数组最大条目数 */);
/* 功能：由外部模块注入电机故障；用途：复用端口层的故障去重和上报队列；无返回值表示事件已尝试记录。 */
void UpperMotorPort_RecordExternalFault(const motor_cfg_t *cfg /**< 当前电机的型号、总线及节点配置 */,
                                        uint8_t error_code /**< 外部模块上报的电机故障码 */,
                                        uint32_t tick_ms /**< 当前系统毫秒时刻 */);
/* 功能：读取当前待上报电机故障；用途：供应用层构造故障消息；返回 true 表示存在尚未确认的故障。 */
bool UpperMotorPort_GetPendingFault(upper_motor_fault_t *fault /**< 用于写出待上报的电机故障记录 */);
/* 功能：确认指定序号的故障已成功发送；用途：释放待上报故障槽；仅序号匹配时清除状态。 */
void UpperMotorPort_MarkFaultSent(uint32_t sequence /**< 用于匹配请求和响应的消息序号 */);
/* 功能：查询指定总线、节点和 DJI 型号是否存在于拓扑；用途：验证调试或诊断路由；返回 true 表示配置匹配。 */
bool UpperMotorPort_IsDjiConfigured(uint8_t can_bus /**< CAN 总线编号 */,
                                    motor_model_t model /**< 待匹配拓扑配置的 DJI 电机型号 */,
                                    uint8_t node_id /**< 电机协议节点编号 */);

#endif
