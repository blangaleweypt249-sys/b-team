/**
 * @file upper_entry.h
 * @brief 声明上层应用初始化、周期控制、通信和遥控接口。
 * @details 所属层：Application/Task；作用：集中声明 UART/CAN、遥控、电机控制
 *          和周期任务接口；上位机服务接口与统计保留在 Driver/PcLink。
 */

#ifndef UPPER_ENTRY_H
#define UPPER_ENTRY_H /**< 防止 upper_entry.h 被重复包含。 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "can_frame.h"
#include "upper_pc_service.h"
#include "upper_remote_link.h"

#define UPPER_J4310_POSITION_KP                  30.0f /**< 上层入口 J4310 位置环默认使用的比例增益。 */
#define UPPER_J4310_POSITION_KD                   0.95f /**< 机械臂 J4310 关节位置环的微分增益。 */

#define UPPER_STALL_ARMING_GRACE_MS                500U /**< 更新位置目标后暂不启用堵转判断的宽限时间，单位：毫秒。 */
#define UPPER_STALL_CONFIRM_MS                    3000U /**< 堵转判据必须连续成立后才确认故障的时间，单位：毫秒。 */
#define UPPER_J4310_STALL_MIN_ERROR_DEG             15.0f /**< 机械臂 J4310 关节判定堵转时要求达到的最小位置误差，单位：度。 */
#define UPPER_J4310_STALL_MAX_VELOCITY_DEG_S         1.0f /**< 机械臂 J4310 关节判定堵转时允许的最大实际速度，单位：度每秒。 */
#define UPPER_J4310_STALL_MIN_TORQUE_NM              3.0f /**< 机械臂 J4310 关节判定堵转时要求达到的最小输出转矩，单位：牛米。 */
#define UPPER_J4310_STALL_RECOVERY_DEG               90.0f /**< 机械臂 J4310 关节发生堵转后执行退让动作的角度，单位：度。 */
#define UPPER_GATE_STALL_MIN_ERROR_DEG               12.0f /**< 挡板机构判定堵转时要求达到的最小位置误差，单位：度。 */
#define UPPER_GATE_STALL_MAX_VELOCITY_DEG_S           1.0f /**< 挡板机构判定堵转时允许的最大实际速度，单位：度每秒。 */
#define UPPER_GATE_STALL_MIN_CURRENT_A                3.0f /**< 挡板机构判定堵转时要求达到的最小输出电流，单位：安培。 */
#define UPPER_GATE_STALL_RECOVERY_DEG                 80.0f /**< 挡板机构发生堵转后执行退让动作的角度，单位：度。 */
#define UPPER_GRIPPER_STALL_MIN_ERROR_DEG             12.0f /**< 夹爪机构判定堵转时要求达到的最小位置误差，单位：度。 */
#define UPPER_GRIPPER_STALL_MAX_VELOCITY_DEG_S         1.0f /**< 夹爪机构判定堵转时允许的最大实际速度，单位：度每秒。 */
#define UPPER_GRIPPER_STALL_MIN_CURRENT_A              3.0f /**< 夹爪机构判定堵转时要求达到的最小输出电流，单位：安培。 */

/* 模式字节 bit0/bit1：PE0=1 选择自动，PD6=1 选择存三。 */
typedef enum
{
    UPPER_REMOTE_MODE_STORE2_MANUAL = 0U, /**< 存二机构流程，由遥控按键手动推进。 */
    UPPER_REMOTE_MODE_STORE3_MANUAL = 1U, /**< 存三机构流程，由遥控按键手动推进。 */
    UPPER_REMOTE_MODE_STORE2_AUTO = 2U, /**< 存二机构流程，由状态机自动推进。 */
    UPPER_REMOTE_MODE_STORE3_AUTO = 3U /**< 存三机构流程，由状态机自动推进。 */
} upper_remote_mode_t;

#define UPPER_REMOTE_STORE2_J4310_STALL_MIN_TORQUE_NM  4.0f /**< 自动存二模式判定 J4310 堵转所需的最小输出转矩，单位：牛米。 */
#define UPPER_REMOTE_STORE2_GATE_STALL_MIN_CURRENT_A    4.0f /**< 挡板机构判定堵转时要求达到的最小输出电流，单位：安培。 */
#define UPPER_REMOTE_STORE2_GRIPPER_STALL_MIN_CURRENT_A 4.0f /**< 夹爪机构判定堵转时要求达到的最小输出电流，单位：安培。 */
#define UPPER_REMOTE_J4310_STALL_MIN_TORQUE_NM(mode /**< 用于选择普通或自动存二阈值的遥控模式 */) \
    (((mode) == UPPER_REMOTE_MODE_STORE2_AUTO) ? \
         UPPER_REMOTE_STORE2_J4310_STALL_MIN_TORQUE_NM : \
         UPPER_J4310_STALL_MIN_TORQUE_NM) /**< 当前遥控模式下 J4310 堵转判定的最小输出转矩，单位：牛米。 */
#define UPPER_REMOTE_GATE_STALL_MIN_CURRENT_A(mode /**< 用于选择普通或自动存二阈值的遥控模式 */) \
    (((mode) == UPPER_REMOTE_MODE_STORE2_AUTO) ? \
         UPPER_REMOTE_STORE2_GATE_STALL_MIN_CURRENT_A : \
         UPPER_GATE_STALL_MIN_CURRENT_A) /**< 当前遥控模式下挡板堵转判定的最小输出电流，单位：安培。 */
#define UPPER_REMOTE_GRIPPER_STALL_MIN_CURRENT_A(mode /**< 用于选择普通或自动存二阈值的遥控模式 */) \
    (((mode) == UPPER_REMOTE_MODE_STORE2_AUTO) ? \
         UPPER_REMOTE_STORE2_GRIPPER_STALL_MIN_CURRENT_A : \
         UPPER_GRIPPER_STALL_MIN_CURRENT_A) /**< 当前遥控模式下夹爪堵转判定的最小输出电流，单位：安培。 */

#define UPPER_REMOTE_MODE_FROM_SWITCHES(primary_switch /**< 主遥控PE0、PD6开关位图 */) \
    ((upper_remote_mode_t)( \
        ((((primary_switch) & UPPER_REMOTE_PRIMARY_SWITCH_PE0) != 0U) ? \
             2U : 0U) | \
        ((((primary_switch) & UPPER_REMOTE_PRIMARY_SWITCH_PD6) != 0U) ? \
             1U : 0U))) /**< 根据主遥控PE0、PD6开关组合得到手动/自动和存二/存三模式。 */

#define UPPER_REMOTE_PD13_FIRST_M3508_DEG       500.0f /**< PD13 动作第一阶段的机械臂 M3508 输出轴目标角度，单位：度。 */
#define UPPER_REMOTE_PD13_FIRST_J4310_DEG        90.0f /**< PD13 动作的第一阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_PD13_SECOND_M3508_DEG     1050.0f /**< PD13 动作的第二阶段机械臂 M3508 输出轴的目标角度，单位：度。 */
#define UPPER_REMOTE_PD13_SECOND_J4310_DEG       90.0f /**< PD13 动作的第二阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_AUTO_PD13_SECOND_J4310_DEG  70.0f /**< 自动存二/存三的 PD13 分支二使用独立的 J4310 目标；手动第二段保持 90 度。 */
#define UPPER_REMOTE_FLIP_PD13_NEXT_M3508_DEG  1000.0f /**< 翻转流程中 PD13 动作下一阶段的机械臂 M3508 输出轴目标角度，单位：度。 */
#define UPPER_REMOTE_FLIP_PD13_NEXT_J4310_DEG    90.0f /**< 翻转流程中 PD13 动作下一阶段的机械臂 J4310 关节目标角度，单位：度。 */
#define UPPER_REMOTE_FLIP_FINAL_M3508_DEG        500.0f /**< 翻转流程中遥控自动流程的收尾阶段机械臂 M3508 输出轴的目标角度，单位：度。 */
#define UPPER_REMOTE_FLIP_FINAL_J4310_DEG         90.0f /**< 翻转流程中遥控自动流程的收尾阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_PD12_FIRST_M3508_DEG         0.0f /**< PD12 动作的第一阶段机械臂 M3508 输出轴的目标角度，单位：度。 */
#define UPPER_REMOTE_PD12_FIRST_J4310_DEG        90.0f /**< PD12 动作的第一阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_PD12_SECOND_M3508_DEG      850.0f /**< PD12 动作的第二阶段机械臂 M3508 输出轴的目标角度，单位：度。 */
#define UPPER_REMOTE_PD12_SECOND_J4310_DEG       90.0f /**< PD12 动作的第二阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_FLIP_PD12_NEXT_M3508_DEG   850.0f /**< 翻转流程中 PD12 动作下一阶段的机械臂 M3508 输出轴目标角度，单位：度。 */
#define UPPER_REMOTE_FLIP_PD12_NEXT_J4310_DEG     90.0f /**< 翻转流程中 PD12 动作下一阶段的机械臂 J4310 关节目标角度，单位：度。 */
#define UPPER_REMOTE_FLIP_FIRST_CLOSE_J4310_DEG    180.0f /**< 翻转流程中遥控自动流程的第一阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_FLIP_FIRST_CLOSE_DELAY_MS        200U /**< 翻转流程第一阶段打开 PE4 后再次关闭 PE4 的等待时间，单位：毫秒。 */
#define UPPER_REMOTE_FLIP_FIRST_FINAL_M3508_DEG     500.0f /**< 翻转流程中遥控自动流程的第一阶段机械臂 M3508 输出轴的目标角度，单位：度。 */
#define UPPER_REMOTE_FLIP_FIRST_FINAL_OPEN_DELAY_MS 1500U /**< 翻转流程第一阶段下发收尾目标后重新打开 PE4 的等待时间，单位：毫秒。 */
#define UPPER_REMOTE_PD11_FIRST_M3508_DEG         0.0f /**< PD11 动作的第一阶段机械臂 M3508 输出轴的目标角度，单位：度。 */
#define UPPER_REMOTE_PD11_FIRST_J4310_DEG       165.0f /**< PD11 动作的第一阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_PD11_SECOND_M3508_DEG        0.0f /**< PD11 动作的第二阶段机械臂 M3508 输出轴的目标角度，单位：度。 */
#define UPPER_REMOTE_PD11_SECOND_J4310_DEG      240.0f /**< PD11 动作的第二阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_PD11_GATE_DEG                40.0f /**< PD11 动作挡板机构的目标角度，单位：度。 */
#define UPPER_REMOTE_STORE2_PD11_GATE_DEG          40.0f /**< 存二模式下 PD11 动作的挡板机构目标角度，单位：度。 */
#define UPPER_REMOTE_STORE2_FINAL_J4310_DEG      180.0f /**< 存二模式下遥控自动流程的收尾阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_STORE2_PD11_DOUBLE_M3508_DEG  1050.0f /**< 存二模式下 PD11 双击动作的机械臂 M3508 输出轴目标角度，单位：度。 */
#define UPPER_REMOTE_STORE2_PD11_DOUBLE_J4310_DEG    90.0f /**< 存二模式下 PD11 双击动作的机械臂 J4310 关节目标角度，单位：度。 */
#define UPPER_REMOTE_PD8_FIRST_M3508_DEG           0.0f /**< PD8 动作的第一阶段机械臂 M3508 输出轴的目标角度，单位：度。 */
#define UPPER_REMOTE_PD8_FIRST_J4310_DEG          40.0f /**< PD8 动作的第一阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_PD8_SECOND_M3508_DEG          0.0f /**< PD8 动作的第二阶段机械臂 M3508 输出轴的目标角度，单位：度。 */
#define UPPER_REMOTE_PD8_SECOND_J4310_DEG         (-20.0f) /**< PD8 动作的第二阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_PD9_ZERO_GATE_DEG               0.0f /**< PD9 动作的复位阶段挡板机构的目标角度，单位：度。 */
#define UPPER_REMOTE_PD9_FIRST_GATE_DEG            180.0f /**< PD9 动作的第一阶段挡板机构的目标角度，单位：度。 */
#define UPPER_REMOTE_PD9_SECOND_GATE_DEG            60.0f /**< PD9 动作的第二阶段挡板机构的目标角度，单位：度。 */
#define UPPER_REMOTE_PC1_GATE_DEG                    80.0f /**< PC1 动作挡板机构的目标角度，单位：度。 */
#define UPPER_REMOTE_PC1_GATE_DISABLE_MIN_DEG        79.0f /**< PC1 动作允许关闭挡板输出的角度区间下限，单位：度。 */
#define UPPER_REMOTE_PC1_GATE_DISABLE_MAX_DEG        81.0f /**< PC1 动作允许关闭挡板输出的角度区间上限，单位：度。 */
#define UPPER_REMOTE_PC0_SECOND_BRANCH_M3508_DEG        0.0f /**< PC0 动作的第二阶段机械臂 M3508 输出轴的目标角度，单位：度。 */
#define UPPER_REMOTE_PC0_THIRD_BRANCH_M3508_DEG       850.0f /**< PC0 第三分支机械臂 M3508 输出轴的目标角度，单位：度。 */
#define UPPER_REMOTE_PC0_FIRST_J4310_DEG              90.0f /**< PC0 动作的第一阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_PC0_CLOSE_M3508_DEG               0.0f /**< PC0 关闭流程机械臂 M3508 输出轴的目标角度，单位：度。 */
#define UPPER_REMOTE_PC0_CLOSE_J4310_DEG              90.0f /**< PC0 动作机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_PC0_SECOND_J4310_DEG            (-20.0f) /**< PC0 动作的第二阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_PC0_GATE_FIRST_DEG               180.0f /**< PC0 动作的第一阶段挡板机构的目标角度，单位：度。 */
#define UPPER_REMOTE_PC0_GATE_FINAL_DEG                68.0f /**< PC0 动作的收尾阶段挡板机构的目标角度，单位：度。 */
#define UPPER_REMOTE_PC0_FINAL_M3508_DEG                0.0f /**< PC0 动作的收尾阶段机械臂 M3508 输出轴的目标角度，单位：度。 */
#define UPPER_REMOTE_PC0_FINAL_J4310_DEG               90.0f /**< PC0 动作的收尾阶段机械臂 J4310 关节的目标角度，单位：度。 */
#define UPPER_REMOTE_PC0_J4310_MATCH_TOLERANCE_DEG      0.1f /**< PC0 动作判断目标到位所允许的角度误差，单位：度。 */
#define UPPER_REMOTE_AUTO_START_GRIPPER_DEG             55.0f /**< 自动模式下遥控自动流程夹爪机构的目标角度，单位：度。 */
#define UPPER_REMOTE_AUTO_START_GRIPPER_STALL_PROTECTION 0U /**< 自动流程启动夹爪初始动作时是否启用堵转保护。 */
#define UPPER_REMOTE_PD10_FIRST_GRIPPER_DEG         55.0f /**< PD10 动作的第一阶段夹爪机构的目标角度，单位：度。 */
#define UPPER_REMOTE_PD10_SECOND_GRIPPER_DEG       135.0f /**< PD10 动作的第二阶段夹爪机构的目标角度，单位：度。 */

#define UPPER_REMOTE_AUTO_START_IS_AVAILABLE( \
    pd13_has_pressed /**< 当前模式是否已执行PD13自动动作 */, \
    pd12_has_pressed /**< 当前模式是否已执行PD12自动动作 */, \
    pc0_has_pressed /**< 当前模式是否已执行PC0自动动作 */) \
    (!((pd13_has_pressed) || (pd12_has_pressed) || (pc0_has_pressed))) /**< PD13、PD12和PC0均未执行时允许启动新的自动流程。 */

#define UPPER_REMOTE_AUTO_HAS_240_STAGE(mode /**< 待判断的遥控模式 */) \
    ((mode) == UPPER_REMOTE_MODE_STORE3_AUTO) /**< 遥控模式是否包含J4310的240度动作阶段。 */
#define UPPER_REMOTE_AUTO_FINAL_J4310_DEG(mode /**< 用于选择收尾角度的遥控模式 */) \
    (((mode) == UPPER_REMOTE_MODE_STORE2_AUTO) ? \
         UPPER_REMOTE_STORE2_FINAL_J4310_DEG : \
         UPPER_REMOTE_PD11_FIRST_J4310_DEG) /**< 当前自动模式收尾阶段的J4310目标角度，单位：度。 */

#define UPPER_REMOTE_FINAL_J4310_DELAY_MS           500U /**< 自动流程下发最终 J4310 目标前的等待时间，单位：毫秒。 */
#define UPPER_REMOTE_PD12_240_HOLD_MS              1500U /**< PD12 动作保持当前机构目标的时间，单位：毫秒。 */
#define UPPER_REMOTE_PD11_OPEN_DELAY_MS            1200U /**< PD11 动作下发首段机构目标后打开 PE4 的等待时间，单位：毫秒。 */
#define UPPER_REMOTE_PD11_J4310_DELAY_MS            500U /**< PD11 动作打开 PE4 后下发 J4310 目标的等待时间，单位：毫秒。 */
#define UPPER_REMOTE_PD11_CLOSE_DELAY_MS           1800U /**< PD11 动作下发 J4310 目标后关闭 PE4 的等待时间，单位：毫秒。 */
#define UPPER_REMOTE_PD11_J4310_AFTER_CLOSE_DELAY_MS 200U /**< PD11 动作关闭 PE4 后下发收尾 J4310 目标的等待时间，单位：毫秒。 */
#define UPPER_REMOTE_STORE2_PD11_OPEN_DELAY_MS      1000U /**< 存二模式下 PD11 动作打开 PE4 前的等待时间，单位：毫秒。 */
#define UPPER_REMOTE_STORE2_PD11_DOUBLE_CLICK_MS     200U /**< 存二模式将两次 PD11 上升沿识别为双击的最大间隔，单位：毫秒。 */
#define UPPER_REMOTE_STORE2_PD11_RETURN_DELAY_MS     1500U /**< 存二模式下 PD11 双击动作回臂前的等待时间，单位：毫秒。 */
#define UPPER_REMOTE_PC0_PD8_DELAY_MS               500U /**< PC0 动作触发 PD8 第二阶段前的等待时间，单位：毫秒。 */
#define UPPER_REMOTE_PC0_FINAL_DELAY_MS                0U /**< PC0 动作进入全局收尾阶段后的等待时间，单位：毫秒。 */

/* 功能：取得遥控第二组控制快照；用途：向上层机构逻辑提供按键、开关和在线状态。 */
bool UpperEntry_GetSecondaryRemoteControl(upper_remote_control_t *control /**< 用于写出当前遥控按键、开关和在线状态的对象 */,
                                          uint32_t tick_ms /**< 当前系统毫秒时刻 */);
/* 功能：读取副遥控链路诊断信息；用途：向监控或调试模块提供收帧统计；无返回值表示诊断数据已复制。 */
void UpperEntry_GetSecondaryRemoteDiagnostics(
    upper_remote_diagnostics_t *diagnostics /**< 用于写出遥控收帧、丢帧及重同步统计的对象 */);

extern volatile uint32_t upper_aux_uart5_sent_count;
extern volatile uint32_t upper_aux_uart5_fail_count;

/* 功能：接收上位机原始字节流；用途：作为外部入口推进链路解析；无返回值表示数据已交给 UpperPcLink。 */
void UpperEntry_OnPcData(const uint8_t *data /**< 上位机链路收到的原始字节流 */,
                         size_t size /**< 本次上位机字节流长度 */,
                         uint32_t tick_ms /**< 当前系统毫秒时刻 */);
/* 功能：接收外部 CAN 帧入口；用途：把电机反馈交给上层电机端口解析；无返回值表示帧已分发。 */
void UpperEntry_OnCanFrame(uint8_t can_bus /**< CAN 总线编号 */,
                           const can_frame_t *frame /**< 待解析的 CAN 接收帧 */,
                           uint32_t tick_ms /**< 当前系统毫秒时刻 */);

/* 功能：初始化上层入口、机器人、链路和通信回调；用途：完成用户应用启动；返回 true 表示所有子模块初始化成功。 */
bool UpperEntry_Init(void);
/* 功能：执行上层应用的 1 ms 主控制周期；用途：处理命令、控制电机、检查故障和通信事件；无返回值表示完成一次调度。 */
void UpperEntry_Control1ms(uint32_t tick_ms /**< 当前系统毫秒时刻 */);
/* 使用通信运行时毫秒时基执行一次上层控制周期。 */
void App_Control1ms(void);

#endif
