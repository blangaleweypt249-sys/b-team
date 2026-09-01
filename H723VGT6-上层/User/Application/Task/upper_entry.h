/**
 * @file upper_entry.h
 * @brief 声明上层应用初始化、周期控制和数据接收接口。
 */

#ifndef UPPER_ENTRY_H
/** 防止 upper_entry.h 被重复包含。 */
#define UPPER_ENTRY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "can_frame.h"
#include "upper_remote_link.h"

/* 上层入口使用的 J4310 默认位置控制增益。 */
/** J4310 位置环默认使用的比例增益。 */
#define UPPER_J4310_POSITION_KP                  30.0f
/** 机械臂 J4310 关节位置环的微分增益。 */
#define UPPER_J4310_POSITION_KD                   0.95f

/* 保守的堵转恢复判据：误差大、速度接近零且输出力度高。 */
/** 更新位置目标后暂不启用堵转判断的宽限时间，单位：毫秒。 */
#define UPPER_STALL_ARMING_GRACE_MS                500U
/** 堵转判据必须连续成立后才确认故障的时间，单位：毫秒。 */
#define UPPER_STALL_CONFIRM_MS                    3000U
/** 机械臂 J4310 关节判定堵转时要求达到的最小位置误差，单位：度。 */
#define UPPER_J4310_STALL_MIN_ERROR_DEG             15.0f
/** 机械臂 J4310 关节判定堵转时允许的最大实际速度，单位：度每秒。 */
#define UPPER_J4310_STALL_MAX_VELOCITY_DEG_S         1.0f
/** 机械臂 J4310 关节判定堵转时要求达到的最小输出转矩，单位：牛米。 */
#define UPPER_J4310_STALL_MIN_TORQUE_NM              3.0f
/** 机械臂 J4310 关节发生堵转后执行退让动作的角度，单位：度。 */
#define UPPER_J4310_STALL_RECOVERY_DEG               90.0f
/** 挡板机构判定堵转时要求达到的最小位置误差，单位：度。 */
#define UPPER_GATE_STALL_MIN_ERROR_DEG               12.0f
/** 挡板机构判定堵转时允许的最大实际速度，单位：度每秒。 */
#define UPPER_GATE_STALL_MAX_VELOCITY_DEG_S           1.0f
/** 挡板机构判定堵转时要求达到的最小输出电流，单位：安培。 */
#define UPPER_GATE_STALL_MIN_CURRENT_A                3.0f
/** 挡板机构发生堵转后执行退让动作的角度，单位：度。 */
#define UPPER_GATE_STALL_RECOVERY_DEG                 80.0f
/** 夹爪机构判定堵转时要求达到的最小位置误差，单位：度。 */
#define UPPER_GRIPPER_STALL_MIN_ERROR_DEG             12.0f
/** 夹爪机构判定堵转时允许的最大实际速度，单位：度每秒。 */
#define UPPER_GRIPPER_STALL_MAX_VELOCITY_DEG_S         1.0f
/** 夹爪机构判定堵转时要求达到的最小输出电流，单位：安培。 */
#define UPPER_GRIPPER_STALL_MIN_CURRENT_A              3.0f

/* 模式字节 bit0/bit1：PE0=1 选择自动，PD6=1 选择存三。 */
typedef enum
{
    UPPER_REMOTE_MODE_STORE2_MANUAL = 0U, /**< 存二机构流程，由遥控按键手动推进。 */
    UPPER_REMOTE_MODE_STORE3_MANUAL = 1U, /**< 存三机构流程，由遥控按键手动推进。 */
    UPPER_REMOTE_MODE_STORE2_AUTO = 2U, /**< 存二机构流程，由状态机自动推进。 */
    UPPER_REMOTE_MODE_STORE3_AUTO = 3U /**< 存三机构流程，由状态机自动推进。 */
} upper_remote_mode_t;

/* 自动存二适当提高出力判据，减少正常负载波动造成的堵转误触发。 */
/** 自动存二模式判定 J4310 堵转所需的最小输出转矩，单位：牛米。 */
#define UPPER_REMOTE_STORE2_J4310_STALL_MIN_TORQUE_NM  4.0f
/** 挡板机构判定堵转时要求达到的最小输出电流，单位：安培。 */
#define UPPER_REMOTE_STORE2_GATE_STALL_MIN_CURRENT_A    4.0f
/** 夹爪机构判定堵转时要求达到的最小输出电流，单位：安培。 */
#define UPPER_REMOTE_STORE2_GRIPPER_STALL_MIN_CURRENT_A 4.0f
/**
 * 机械臂 J4310 关节判定堵转时要求达到的最小输出转矩，单位：牛米。
 * @param mode 需要设置或判断的工作模式。
 */
#define UPPER_REMOTE_J4310_STALL_MIN_TORQUE_NM(mode) \
    (((mode) == UPPER_REMOTE_MODE_STORE2_AUTO) ? \
         UPPER_REMOTE_STORE2_J4310_STALL_MIN_TORQUE_NM : \
         UPPER_J4310_STALL_MIN_TORQUE_NM)
/**
 * 挡板机构判定堵转时要求达到的最小输出电流，单位：安培。
 * @param mode 需要设置或判断的工作模式。
 */
#define UPPER_REMOTE_GATE_STALL_MIN_CURRENT_A(mode) \
    (((mode) == UPPER_REMOTE_MODE_STORE2_AUTO) ? \
         UPPER_REMOTE_STORE2_GATE_STALL_MIN_CURRENT_A : \
         UPPER_GATE_STALL_MIN_CURRENT_A)
/**
 * 夹爪机构判定堵转时要求达到的最小输出电流，单位：安培。
 * @param mode 需要设置或判断的工作模式。
 */
#define UPPER_REMOTE_GRIPPER_STALL_MIN_CURRENT_A(mode) \
    (((mode) == UPPER_REMOTE_MODE_STORE2_AUTO) ? \
         UPPER_REMOTE_STORE2_GRIPPER_STALL_MIN_CURRENT_A : \
         UPPER_GRIPPER_STALL_MIN_CURRENT_A)

/**
 * 根据当前遥控输入或模式计算对应动作条件。
 * @param primary_switch 主遥控 PE0、PD6 开关的当前位图。
 */
#define UPPER_REMOTE_MODE_FROM_SWITCHES(primary_switch) \
    ((upper_remote_mode_t)( \
        ((((primary_switch) & UPPER_REMOTE_PRIMARY_SWITCH_PE0) != 0U) ? \
             2U : 0U) | \
        ((((primary_switch) & UPPER_REMOTE_PRIMARY_SWITCH_PD6) != 0U) ? \
             1U : 0U)))

/* 遥控动作的物理目标，单位为机构输出角度（度）。 */
/** PD13 动作第一阶段的机械臂 M3508 输出轴目标角度，单位：度。 */
#define UPPER_REMOTE_PD13_FIRST_M3508_DEG       500.0f
/** PD13 动作的第一阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_PD13_FIRST_J4310_DEG        90.0f
/** PD13 动作的第二阶段机械臂 M3508 输出轴的目标角度，单位：度。 */
#define UPPER_REMOTE_PD13_SECOND_M3508_DEG     1050.0f
/** PD13 动作的第二阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_PD13_SECOND_J4310_DEG       90.0f
/* 自动存二/存三的 PD13 分支二使用独立的 J4310 目标；手动第二段保持 90 度。 */
#define UPPER_REMOTE_AUTO_PD13_SECOND_J4310_DEG  70.0f
/** 翻转流程中 PD13 动作下一阶段的机械臂 M3508 输出轴目标角度，单位：度。 */
#define UPPER_REMOTE_FLIP_PD13_NEXT_M3508_DEG  1000.0f
/** 翻转流程中 PD13 动作下一阶段的机械臂 J4310 关节目标角度，单位：度。 */
#define UPPER_REMOTE_FLIP_PD13_NEXT_J4310_DEG    90.0f
/** 翻转流程中遥控自动流程的收尾阶段机械臂 M3508 输出轴的目标角度，单位：度。 */
#define UPPER_REMOTE_FLIP_FINAL_M3508_DEG        500.0f
/** 翻转流程中遥控自动流程的收尾阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_FLIP_FINAL_J4310_DEG         90.0f
/** PD12 动作的第一阶段机械臂 M3508 输出轴的目标角度，单位：度。 */
#define UPPER_REMOTE_PD12_FIRST_M3508_DEG         0.0f
/** PD12 动作的第一阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_PD12_FIRST_J4310_DEG        90.0f
/** PD12 动作的第二阶段机械臂 M3508 输出轴的目标角度，单位：度。 */
#define UPPER_REMOTE_PD12_SECOND_M3508_DEG      850.0f
/** PD12 动作的第二阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_PD12_SECOND_J4310_DEG       90.0f
/** 翻转流程中 PD12 动作下一阶段的机械臂 M3508 输出轴目标角度，单位：度。 */
#define UPPER_REMOTE_FLIP_PD12_NEXT_M3508_DEG   850.0f
/** 翻转流程中 PD12 动作下一阶段的机械臂 J4310 关节目标角度，单位：度。 */
#define UPPER_REMOTE_FLIP_PD12_NEXT_J4310_DEG     90.0f
/** 翻转流程中遥控自动流程的第一阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_FLIP_FIRST_CLOSE_J4310_DEG    180.0f
/** 翻转流程中遥控自动流程进入下一阶段前的等待时间，单位：毫秒。 */
#define UPPER_REMOTE_FLIP_FIRST_CLOSE_DELAY_MS        200U
/** 翻转流程中遥控自动流程的第一阶段机械臂 M3508 输出轴的目标角度，单位：度。 */
#define UPPER_REMOTE_FLIP_FIRST_FINAL_M3508_DEG     500.0f
/** 翻转流程中遥控自动流程进入下一阶段前的等待时间，单位：毫秒。 */
#define UPPER_REMOTE_FLIP_FIRST_FINAL_OPEN_DELAY_MS 1500U
/** PD11 动作的第一阶段机械臂 M3508 输出轴的目标角度，单位：度。 */
#define UPPER_REMOTE_PD11_FIRST_M3508_DEG         0.0f
/** PD11 动作的第一阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_PD11_FIRST_J4310_DEG       165.0f
/** PD11 动作的第二阶段机械臂 M3508 输出轴的目标角度，单位：度。 */
#define UPPER_REMOTE_PD11_SECOND_M3508_DEG        0.0f
/** PD11 动作的第二阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_PD11_SECOND_J4310_DEG      240.0f
/** PD11 动作挡板机构的目标角度，单位：度。 */
#define UPPER_REMOTE_PD11_GATE_DEG                40.0f
/** 存二模式下 PD11 动作的挡板机构目标角度，单位：度。 */
#define UPPER_REMOTE_STORE2_PD11_GATE_DEG          40.0f
/** 存二模式下遥控自动流程的收尾阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_STORE2_FINAL_J4310_DEG      180.0f
/** 存二模式下 PD11 双击动作的机械臂 M3508 输出轴目标角度，单位：度。 */
#define UPPER_REMOTE_STORE2_PD11_DOUBLE_M3508_DEG  1050.0f
/** 存二模式下 PD11 双击动作的机械臂 J4310 关节目标角度，单位：度。 */
#define UPPER_REMOTE_STORE2_PD11_DOUBLE_J4310_DEG    90.0f
/** PD8 动作的第一阶段机械臂 M3508 输出轴的目标角度，单位：度。 */
#define UPPER_REMOTE_PD8_FIRST_M3508_DEG           0.0f
/** PD8 动作的第一阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_PD8_FIRST_J4310_DEG          40.0f
/** PD8 动作的第二阶段机械臂 M3508 输出轴的目标角度，单位：度。 */
#define UPPER_REMOTE_PD8_SECOND_M3508_DEG          0.0f
/** PD8 动作的第二阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_PD8_SECOND_J4310_DEG         (-20.0f)
/** PD9 动作的复位阶段挡板机构的目标角度，单位：度。 */
#define UPPER_REMOTE_PD9_ZERO_GATE_DEG               0.0f
/** PD9 动作的第一阶段挡板机构的目标角度，单位：度。 */
#define UPPER_REMOTE_PD9_FIRST_GATE_DEG            180.0f
/** PD9 动作的第二阶段挡板机构的目标角度，单位：度。 */
#define UPPER_REMOTE_PD9_SECOND_GATE_DEG            60.0f
/** PC1 动作挡板机构的目标角度，单位：度。 */
#define UPPER_REMOTE_PC1_GATE_DEG                    80.0f
/** PC1 动作允许关闭挡板输出的角度区间下限，单位：度。 */
#define UPPER_REMOTE_PC1_GATE_DISABLE_MIN_DEG        79.0f
/** PC1 动作允许关闭挡板输出的角度区间上限，单位：度。 */
#define UPPER_REMOTE_PC1_GATE_DISABLE_MAX_DEG        81.0f
/** PC0 动作的第二阶段机械臂 M3508 输出轴的目标角度，单位：度。 */
#define UPPER_REMOTE_PC0_SECOND_BRANCH_M3508_DEG        0.0f
/** PC0 动作机械臂 M3508 输出轴的目标角度，单位：度。 */
#define UPPER_REMOTE_PC0_THIRD_BRANCH_M3508_DEG       850.0f
/** PC0 动作的第一阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_PC0_FIRST_J4310_DEG              90.0f
/** PC0 动作机械臂 M3508 输出轴的目标角度，单位：度。 */
#define UPPER_REMOTE_PC0_CLOSE_M3508_DEG               0.0f
/** PC0 动作机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_PC0_CLOSE_J4310_DEG              90.0f
/** PC0 动作的第二阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_PC0_SECOND_J4310_DEG            (-20.0f)
/** PC0 动作的第一阶段挡板机构的目标角度，单位：度。 */
#define UPPER_REMOTE_PC0_GATE_FIRST_DEG               180.0f
/** PC0 动作的收尾阶段挡板机构的目标角度，单位：度。 */
#define UPPER_REMOTE_PC0_GATE_FINAL_DEG                68.0f
/** PC0 动作的收尾阶段机械臂 M3508 输出轴的目标角度，单位：度。 */
#define UPPER_REMOTE_PC0_FINAL_M3508_DEG                0.0f
/** PC0 动作的收尾阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_PC0_FINAL_J4310_DEG               90.0f
/** PC0 动作判断目标到位所允许的角度误差，单位：度。 */
#define UPPER_REMOTE_PC0_J4310_MATCH_TOLERANCE_DEG      0.1f
/** 自动模式下遥控自动流程夹爪机构的目标角度，单位：度。 */
#define UPPER_REMOTE_AUTO_START_GRIPPER_DEG             55.0f
/** 自动流程启动夹爪初始动作时是否启用堵转保护。 */
#define UPPER_REMOTE_AUTO_START_GRIPPER_STALL_PROTECTION 0U
/** PD10 动作的第一阶段夹爪机构的目标角度，单位：度。 */
#define UPPER_REMOTE_PD10_FIRST_GRIPPER_DEG         55.0f
/** PD10 动作的第二阶段夹爪机构的目标角度，单位：度。 */
#define UPPER_REMOTE_PD10_SECOND_GRIPPER_DEG       135.0f

/**
 * 判断 PD13、PD12 和 PC0 自动动作历史是否均为空，从而允许启动新的自动流程。
 * @param pd13_has_pressed 当前模式中是否已经执行过 PD13 自动动作。
 * @param pd12_has_pressed 当前模式中是否已经执行过 PD12 自动动作。
 * @param pc0_has_pressed 当前模式中是否已经执行过 PC0 自动动作。
 */
#define UPPER_REMOTE_AUTO_START_IS_AVAILABLE( \
    pd13_has_pressed, pd12_has_pressed, pc0_has_pressed) \
    (!((pd13_has_pressed) || (pd12_has_pressed) || (pc0_has_pressed)))

/**
 * 根据当前遥控输入或模式计算对应动作条件。
 * @param mode 需要设置或判断的工作模式。
 */
#define UPPER_REMOTE_AUTO_HAS_240_STAGE(mode) \
    ((mode) == UPPER_REMOTE_MODE_STORE3_AUTO)
/**
 * 自动模式下遥控自动流程的收尾阶段机械臂 J4310 关节的目标角度，单位：度。
 * @param mode 需要设置或判断的工作模式。
 */
#define UPPER_REMOTE_AUTO_FINAL_J4310_DEG(mode) \
    (((mode) == UPPER_REMOTE_MODE_STORE2_AUTO) ? \
         UPPER_REMOTE_STORE2_FINAL_J4310_DEG : \
         UPPER_REMOTE_PD11_FIRST_J4310_DEG)

/* PD12 的 1500 ms 仅用于存三自动发送 240 度后的保持阶段。 */
/** 自动流程下发最终 J4310 目标前的等待时间，单位：毫秒。 */
#define UPPER_REMOTE_FINAL_J4310_DELAY_MS           500U
/** PD12 动作保持当前机构目标的时间，单位：毫秒。 */
#define UPPER_REMOTE_PD12_240_HOLD_MS              1500U
/** PD11 动作进入下一阶段前的等待时间，单位：毫秒。 */
#define UPPER_REMOTE_PD11_OPEN_DELAY_MS            1200U
/** PD11 动作进入下一阶段前的等待时间，单位：毫秒。 */
#define UPPER_REMOTE_PD11_J4310_DELAY_MS            500U
/** PD11 动作进入下一阶段前的等待时间，单位：毫秒。 */
#define UPPER_REMOTE_PD11_CLOSE_DELAY_MS           1800U
/** PD11 动作进入下一阶段前的等待时间，单位：毫秒。 */
#define UPPER_REMOTE_PD11_J4310_AFTER_CLOSE_DELAY_MS 200U
/** 存二模式下 PD11 动作打开 PE4 前的等待时间，单位：毫秒。 */
#define UPPER_REMOTE_STORE2_PD11_OPEN_DELAY_MS      1000U
/** 存二模式将两次 PD11 上升沿识别为双击的最大间隔，单位：毫秒。 */
#define UPPER_REMOTE_STORE2_PD11_DOUBLE_CLICK_MS     200U
/** 存二模式下 PD11 双击动作回臂前的等待时间，单位：毫秒。 */
#define UPPER_REMOTE_STORE2_PD11_RETURN_DELAY_MS     1500U
/** PC0 动作进入下一阶段前的等待时间，单位：毫秒。 */
#define UPPER_REMOTE_PC0_PD8_DELAY_MS               500U
/** PC0 动作进入下一阶段前的等待时间，单位：毫秒。 */
#define UPPER_REMOTE_PC0_FINAL_DELAY_MS                0U

extern volatile uint32_t upper_handshake_ack_sent_count;
extern volatile uint32_t upper_handshake_ack_busy_count;
extern volatile uint32_t upper_handshake_ack_fail_count;
extern volatile uint32_t upper_state_sent_count;
extern volatile uint32_t upper_state_busy_count;
extern volatile uint32_t upper_state_fail_count;
extern volatile uint32_t upper_dji_telemetry_sent_count;
extern volatile uint32_t upper_dji_telemetry_busy_count;
extern volatile uint32_t upper_dji_telemetry_fail_count;
extern volatile uint32_t upper_aux_uart5_sent_count;
extern volatile uint32_t upper_aux_uart5_fail_count;
/* 功能：初始化上层入口、机器人、链路和通信回调；用途：完成用户应用启动；返回 true 表示所有子模块初始化成功。 */
bool UpperEntry_Init(void);
/* 功能：执行上层应用的 1 ms 主控制周期；用途：处理命令、控制电机、检查故障和通信事件；无返回值表示完成一次调度。 */
void UpperEntry_Control1ms(uint32_t tick_ms /* 当前系统毫秒时刻 */);
/* 功能：接收上位机原始字节流；用途：作为外部入口推进链路解析；无返回值表示数据已交给 UpperPcLink。 */
void UpperEntry_OnPcData(const uint8_t *data /* 待处理数据的首地址 */,
                         size_t size /* 待处理数据的字节数 */,
                         uint32_t tick_ms /* 当前系统毫秒时刻 */);
/* 功能：接收外部 CAN 帧入口；用途：把电机反馈交给上层电机端口解析；无返回值表示帧已分发。 */
void UpperEntry_OnCanFrame(uint8_t can_bus /* CAN 总线编号 */,
                           const can_frame_t *frame /* 需要解析或发送的 CAN 或协议帧 */,
                           uint32_t tick_ms /* 当前系统毫秒时刻 */);
/* 功能：取得遥控第二组控制快照；用途：向上层机构逻辑提供按键、开关和在线状态。 */
bool UpperEntry_GetSecondaryRemoteControl(upper_remote_control_t *control /* 需要读取或更新的控制状态 */,
                                          uint32_t tick_ms /* 当前系统毫秒时刻 */);
/* 功能：读取副遥控链路诊断信息；用途：向监控或调试模块提供收帧统计；无返回值表示诊断数据已复制。 */
void UpperEntry_GetSecondaryRemoteDiagnostics(
    upper_remote_diagnostics_t *diagnostics /* 用于写出诊断统计的对象 */);

#endif
