/**
 * @file upper_entry.h
 * @brief 声明上层应用初始化、周期控制和数据接收接口。
 */

#ifndef UPPER_ENTRY_H
#define UPPER_ENTRY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "can_frame.h"
#include "upper_remote_link.h"

/* 上层入口使用的 J4310 默认位置控制增益。 */
#define UPPER_J4310_POSITION_KP                  20.0f
#define UPPER_J4310_POSITION_KD                   0.95f

/* 遥控动作的物理目标，单位为机构输出角度（度）。 */
#define UPPER_REMOTE_PD13_FIRST_M3508_DEG       500.0f
#define UPPER_REMOTE_PD13_FIRST_J4310_DEG        90.0f
#define UPPER_REMOTE_PD13_SECOND_M3508_DEG     1000.0f
#define UPPER_REMOTE_PD13_SECOND_J4310_DEG       90.0f
#define UPPER_REMOTE_PD12_FIRST_M3508_DEG         0.0f
#define UPPER_REMOTE_PD12_FIRST_J4310_DEG        90.0f
#define UPPER_REMOTE_PD12_SECOND_M3508_DEG      850.0f
#define UPPER_REMOTE_PD12_SECOND_J4310_DEG       90.0f
#define UPPER_REMOTE_PD11_FIRST_M3508_DEG         0.0f
#define UPPER_REMOTE_PD11_FIRST_J4310_DEG       180.0f
#define UPPER_REMOTE_PD8_FIRST_M3508_DEG           0.0f
#define UPPER_REMOTE_PD8_FIRST_J4310_DEG          40.0f
#define UPPER_REMOTE_PD8_SECOND_M3508_DEG           0.0f
#define UPPER_REMOTE_PD8_SECOND_J4310_DEG         (-10.0f)
#define UPPER_REMOTE_PD9_ZERO_GATE_DEG               0.0f
#define UPPER_REMOTE_PD9_FIRST_GATE_DEG            180.0f
#define UPPER_REMOTE_PD9_SECOND_GATE_DEG            60.0f
#define UPPER_REMOTE_PD9_OSCILLATION_HIGH_DEG      130.0f
#define UPPER_REMOTE_PD9_OSCILLATION_LOW_DEG        55.0f
#define UPPER_REMOTE_PD10_FIRST_GRIPPER_DEG        125.0f
#define UPPER_REMOTE_PD10_SECOND_GRIPPER_DEG        45.0f

extern volatile uint32_t upper_handshake_ack_sent_count;
extern volatile uint32_t upper_handshake_ack_busy_count;
extern volatile uint32_t upper_handshake_ack_fail_count;
extern volatile uint32_t upper_state_sent_count;
extern volatile uint32_t upper_state_busy_count;
extern volatile uint32_t upper_state_fail_count;
extern volatile uint32_t upper_dji_telemetry_sent_count;
extern volatile uint32_t upper_dji_telemetry_busy_count;
extern volatile uint32_t upper_dji_telemetry_fail_count;
extern volatile uint32_t upper_aux_spi3_sent_count;
extern volatile uint32_t upper_aux_spi3_fail_count;

/* 功能：初始化上层入口、机器人、链路和通信回调；用途：完成用户应用启动；返回 true 表示所有子模块初始化成功。 */
bool UpperEntry_Init(void);
/* 功能：执行上层应用的 1 ms 主控制周期；用途：处理命令、控制电机、检查故障并发送状态；无返回值表示完成一次调度。 */
void UpperEntry_Control1ms(uint32_t tick_ms);
/* 功能：接收上位机原始字节流；用途：作为外部入口推进链路解析；无返回值表示数据已交给 UpperPcLink。 */
void UpperEntry_OnPcData(const uint8_t *data,
                         size_t size,
                         uint32_t tick_ms);
/* 功能：接收外部 CAN 帧入口；用途：把电机反馈交给上层电机端口解析；无返回值表示帧已分发。 */
void UpperEntry_OnCanFrame(uint8_t can_bus,
                           const can_frame_t *frame,
                           uint32_t tick_ms);
/* 功能：取得遥控第二组控制快照；用途：向上层机构逻辑提供按键、开关和在线状态。 */
bool UpperEntry_GetSecondaryRemoteControl(upper_remote_control_t *control,
                                          uint32_t tick_ms);
/* 功能：读取副遥控链路诊断信息；用途：向监控或调试模块提供收帧统计；无返回值表示诊断数据已复制。 */
void UpperEntry_GetSecondaryRemoteDiagnostics(
    upper_remote_diagnostics_t *diagnostics);

#endif
