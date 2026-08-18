/**
 * @file upper_motor_port.h
 * @brief 声明上层电机端口、反馈诊断和故障上报接口。
 */

#ifndef UPPER_MOTOR_PORT_H
#define UPPER_MOTOR_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "can_frame.h"
#include "motor_manager.h"

typedef enum
{
    UPPER_MOTOR_ARM_M3508_1,
    UPPER_MOTOR_ARM_M3508_2,
    UPPER_MOTOR_ARM_J4310,
    UPPER_MOTOR_GATE_M2006,
    UPPER_MOTOR_GRIPPER_M2006,
    UPPER_MOTOR_COUNT
} upper_motor_id_t;

typedef struct
{
    uint32_t active_mask;
    uint32_t offline_mask;
    uint32_t fault_mask;
    uint32_t protocol_block_mask;
} upper_motor_health_t;

typedef struct
{
    motor_model_t model;
    uint8_t can_bus;
    uint8_t node_id;
    uint8_t error_code;
    uint32_t tick_ms;
    uint32_t sequence;
} upper_motor_fault_t;

typedef struct
{
    motor_model_t model;
    uint8_t can_bus;
    uint8_t node_id;
    bool feedback_received;
    bool zero_valid;
    bool feedback_fresh;
    float rotor_position_rad;
    float zero_rotor_position_rad;
    float relative_output_position_rad;
} upper_dji_diagnostic_t;

typedef struct
{
    uint32_t frames_seen;
    uint32_t accepted_frames;
    uint32_t rejected_format_frames;
    uint32_t rejected_master_id_frames;
    uint32_t rejected_feedback_id_frames;
    uint16_t last_can_id;
    uint8_t last_dlc;
    uint8_t last_data0;
    uint8_t last_result;
} upper_j4310_rx_diagnostic_t;

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
} upper_j4310_tx_diagnostic_t;

typedef struct
{
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    uint32_t updated_at_ms;
    uint8_t state;
} upper_j4310_feedback_t;

#define UPPER_MOTOR_ERROR_FEEDBACK_TIMEOUT 0xF0U

/* 功能：初始化电机协议驱动、拓扑状态和故障状态；用途：建立上层统一电机端口；返回 true 表示全部型号初始化成功。 */
bool UpperMotorPort_Init(const motor_cfg_t *cfg, size_t motor_count);
/* 功能：开始新的电机发送周期并清空 DJI 分组暂存；用途：保证各槽位只包含本周期命令；无返回值表示周期状态已复位。 */
void UpperMotorPort_BeginCycle(uint32_t tick_ms);
/* 功能：按配置型号路由统一电机命令；用途：作为 MotorManager 的发送回调；返回 true 表示命令被对应协议接受。 */
bool UpperMotorPort_Send(const motor_cfg_t *cfg,
                         const motor_cmd_t *cmd,
                         void *user_data);
/* 功能：发送本周期所有待处理 DJI 分组帧；用途：在各电机目标均暂存后统一输出；返回 true 表示所有组发送成功。 */
bool UpperMotorPort_Flush(void);
/* 功能：按总线和标识符分发电机反馈帧；用途：更新 J4310、M3508 或 M2006 驱动状态；无返回值表示已尝试路由。 */
void UpperMotorPort_OnFrame(uint8_t can_bus,
                             const can_frame_t *frame,
                             uint32_t tick_ms);
/* 功能：检查全部电机的反馈超时和协议故障；用途：形成整机健康结论并记录新故障；返回 true 表示所有电机健康。 */
bool UpperMotorPort_GetHealth(uint32_t tick_ms,
                               upper_motor_health_t *health);
/* 功能：仅发送指定 J4310 的协议使能帧；用途：上电恢复时先使能电机而不下发运动目标；返回 true 表示使能帧发送成功。 */
bool UpperMotorPort_EnableJ4310(uint8_t can_bus, uint8_t node_id);
/* 功能：校验反馈并执行 J4310 保存零点序列；用途：安全完成关节机械零位标定；返回 true 表示命令序列发送成功。 */
bool UpperMotorPort_SaveJ4310Zero(uint8_t can_bus, uint8_t node_id);
/* 功能：读取 J4310 经方向和减速比换算后的输出轴位置；用途：向应用层提供机械关节角；返回 true 表示反馈新鲜有效。 */
bool UpperMotorPort_GetJ4310OutputPosition(uint8_t can_bus,
                                           uint8_t node_id,
                                           float *position_rad);
/* 功能：读取 J4310 自动归零所需的位置、速度和状态；用途：仅向上层提供新鲜且无故障的反馈；返回 true 表示可安全用于轨迹控制。 */
bool UpperMotorPort_GetJ4310Feedback(uint8_t can_bus,
                                     uint8_t node_id,
                                     upper_j4310_feedback_t *feedback);
/* 功能：读取 J4310 协议接收诊断；用途：区分 FDCAN 有帧但格式、Master ID 或反馈 ID 不匹配；返回 true 表示目标已配置且快照有效。 */
bool UpperMotorPort_GetJ4310RxDiagnostic(
    uint8_t can_bus,
    uint8_t node_id,
    upper_j4310_rx_diagnostic_t *diagnostic);
/* 功能：读取 J4310 发送与使能确认快照；用途：向上位机区分未发送、TX 入队失败和电机未确认；返回 true 表示节点已配置。 */
bool UpperMotorPort_GetJ4310TxDiagnostic(
    uint8_t can_bus,
    uint8_t node_id,
    upper_j4310_tx_diagnostic_t *diagnostic);
/* 功能：收集已配置 DJI 电机的诊断快照；用途：上报转子位置、零点、输出位置和反馈新鲜度；返回值表示写入条目数。 */
size_t UpperMotorPort_GetDjiDiagnostics(uint32_t tick_ms,
                                        upper_dji_diagnostic_t *diagnostics,
                                        size_t capacity);
/* 功能：由外部模块注入电机故障；用途：复用端口层的故障去重和上报队列；无返回值表示事件已尝试记录。 */
void UpperMotorPort_RecordExternalFault(const motor_cfg_t *cfg,
                                        uint8_t error_code,
                                        uint32_t tick_ms);
/* 功能：读取当前待上报电机故障；用途：供应用层构造故障消息；返回 true 表示存在尚未确认的故障。 */
bool UpperMotorPort_GetPendingFault(upper_motor_fault_t *fault);
/* 功能：确认指定序号的故障已成功发送；用途：释放待上报故障槽；仅序号匹配时清除状态。 */
void UpperMotorPort_MarkFaultSent(uint32_t sequence);
/* 功能：查询指定总线、节点和 DJI 型号是否存在于拓扑；用途：验证调试或诊断路由；返回 true 表示配置匹配。 */
bool UpperMotorPort_IsDjiConfigured(uint8_t can_bus,
                                    motor_model_t model,
                                    uint8_t node_id);

#endif
