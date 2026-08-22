/**
 * @file upper_entry.c
 * @brief 实现上层应用入口，协调通信、遥控、电机和周期任务。
 */

#include "upper_entry.h"

#include <string.h>

#include "arm.h"
#include "bsp_can.h"
#include "cmsis_os2.h"
#include "comm_runtime.h"
#include "j4310_auto_return.h"
#include "j4310_position_control.h"
#include "position_stall_monitor.h"
#include "upper_motor_port.h"
#include "upper_pc_link.h"
#include "upper_remote_link.h"
#include "upper_robot.h"
#include "W25Qxx.h"

#define UPPER_CMD_QUEUE_DEPTH  4U
#define UPPER_STATE_PERIOD_MS  50U
#define UPPER_DJI_TELEMETRY_PERIOD_MS 10U
#define UPPER_TX_BUFFER_SIZE   160U
#define UPPER_HANDSHAKE_ACK_GUARD_MS 20U
#define UPPER_J4310_STARTUP_ENABLE_RETRY_MS 20U
#define UPPER_REMOTE_KEY_PD13        (1U << 0U)
#define UPPER_REMOTE_KEY_PD12        (1U << 1U)
#define UPPER_REMOTE_KEY_PD11        (1U << 2U)
#define UPPER_REMOTE_KEY_PD8         (1U << 3U)
#define UPPER_REMOTE_KEY_PD9         (1U << 4U)
#define UPPER_REMOTE_KEY_PD10        (1U << 5U)
#define UPPER_REMOTE_SWITCH_PE4      (1U << 0U)
#define UPPER_AUX_OUTPUT_PE4         (1U << 0U)
#define UPPER_AUX_OUTPUT_MASK        0x0FU
#define UPPER_AUX_UPDATE_SHIFT       4U
#define UPPER_REMOTE_ANGLE_DEG_TO_RAD 0.017453292519943295f
#define UPPER_REMOTE_GRIPPER_MOTOR_DEG_PER_OUTPUT_DEG 2.0f
#define UPPER_AUX_UART_FRAME_SIZE       8U
#define UPPER_AUX_UART_TARGET_RECEIVER  0x01U
#define UPPER_AUX_UART_TYPE_CONTROL     0x02U
#define UPPER_AUX_UART_PAYLOAD_SIZE     1U
#define UPPER_AUX_UART_REPEAT_COUNT     3U
#define UPPER_CONTROL_PERIOD_MS          1U
#define UPPER_REMOTE_MODE_COUNT          4U

#define UPPER_J4310_TRAJECTORY_MAX_VEL_RAD_S       3.7f
#define UPPER_J4310_TRAJECTORY_MAX_ACCEL_RAD_S2   15.0f
#define UPPER_J4310_GRAVITY_MODEL_LIMIT_NM          8.0f
#define UPPER_J4310_GRAVITY_LEARNING_RATE           0.01f
#define UPPER_J4310_GRAVITY_COMPENSATION_GAIN       1.0f
#define UPPER_J4310_GRAVITY_DISABLE_HALF_WIDTH_RAD  0.0872664626f
#define UPPER_J4310_GRAVITY_SETTLE_ERROR_RAD         0.01745329252f
#define UPPER_J4310_GRAVITY_SETTLE_FEEDBACK_COUNT  100U
#define UPPER_J4310_GRAVITY_TORQUE_RATE_NM_S         3.0f

#define UPPER_REMOTE_PD13_RESET_KEYS \
    (UPPER_REMOTE_KEY_PD12 | UPPER_REMOTE_KEY_PD11 | \
     UPPER_REMOTE_KEY_PD8)
#define UPPER_REMOTE_PD12_RESET_KEYS \
    (UPPER_REMOTE_KEY_PD13 | UPPER_REMOTE_KEY_PD11 | \
     UPPER_REMOTE_KEY_PD8)
#define UPPER_REMOTE_PD8_RESET_KEYS \
    (UPPER_REMOTE_KEY_PD13 | UPPER_REMOTE_KEY_PD12 | \
     UPPER_REMOTE_KEY_PD11)
#define UPPER_REMOTE_PD11_RESET_KEYS \
    (UPPER_REMOTE_KEY_PD13 | UPPER_REMOTE_KEY_PD12 | \
     UPPER_REMOTE_KEY_PD8)

typedef enum
{
    UPPER_REMOTE_AUTO_PE4_IDLE = 0,             /* 翻转子流程空闲 */
    UPPER_REMOTE_AUTO_PE4_WAIT_RESET,           /* 等待首次手动关闭 PE4 */
    UPPER_REMOTE_AUTO_PE4_WAIT_PD13,            /* 等待 PD13 确认收尾 */
    UPPER_REMOTE_AUTO_PE4_WAIT_PD12,            /* 等待 PD12 确认收尾 */
    UPPER_REMOTE_AUTO_PE4_WAIT_FIRST_CLOSE_DELAY, /* 等待首段关闭延时 */
    UPPER_REMOTE_AUTO_PE4_WAIT_FINAL_OPEN_DELAY /* 等待 J4310=40 后打开 PE4 */
} upper_remote_auto_pe4_state_t;

typedef enum
{
    UPPER_REMOTE_FLIP_ACTION_NONE = 0,
    UPPER_REMOTE_FLIP_ACTION_PD13,
    UPPER_REMOTE_FLIP_ACTION_PD12
} upper_remote_flip_action_t;

typedef enum
{
    UPPER_REMOTE_AUTO_PD13_IDLE = 0,                    /* PD13 自动流程空闲 */
    UPPER_REMOTE_AUTO_PD13_WAIT_FIRST_PE4,              /* 分支一等待手动关闭 PE4 */
    UPPER_REMOTE_AUTO_PD13_WAIT_BRANCH_ONE_FINAL_J4310, /* 分支一等待 500 ms 后发送模式收尾角度 */
    UPPER_REMOTE_AUTO_PD13_WAIT_DIRECT_SECOND_PRESS,    /* 分支一已结束，下一按进入分支二 */
    UPPER_REMOTE_AUTO_PD13_WAIT_DIRECT_SECOND_PE4,      /* 分支二等待 PE4 关闭条件 */
    UPPER_REMOTE_AUTO_PD13_WAIT_DIRECT_SECOND_J4310     /* 分支二等待 500 ms 后下发自动收尾角度 */
} upper_remote_auto_pd13_state_t;

typedef enum
{
    UPPER_REMOTE_AUTO_PD12_IDLE = 0,                          /* PD12 自动流程空闲 */
    UPPER_REMOTE_AUTO_PD12_WAIT_FIRST_PE4_OR_SECOND_PRESS,    /* 等待第一次 PE4 或第二次按下 */
    UPPER_REMOTE_AUTO_PD12_WAIT_PD11_J4310,                   /* 等待 PD11 流程中的 J4310 动作 */
    UPPER_REMOTE_AUTO_PD12_WAIT_AUTO_RESET_DELAY,             /* 等待自动复位延时 */
    UPPER_REMOTE_AUTO_PD12_WAIT_FINAL_PE4,                    /* 等待最终 PE4 操作 */
    UPPER_REMOTE_AUTO_PD12_WAIT_DIRECT_SECOND_PRESS,          /* 分支一已结束，下一按进入分支二 */
    UPPER_REMOTE_AUTO_PD12_WAIT_DIRECT_SECOND_PE4,            /* 分支二等待 PE4 关闭条件 */
    UPPER_REMOTE_AUTO_PD12_WAIT_BRANCH_ONE_FINAL_J4310,       /* 分支一等待 500 ms 后下发自动收尾角度 */
    UPPER_REMOTE_AUTO_PD12_WAIT_BRANCH_TWO_FINAL_J4310        /* 分支二等待 500 ms 后下发自动收尾角度 */
} upper_remote_auto_pd12_state_t;

typedef enum
{
    UPPER_REMOTE_AUTO_PD11_IDLE = 0,          /* PD11 自动流程空闲 */
    UPPER_REMOTE_AUTO_PD11_WAIT_SECOND_CLICK, /* 存二等待双击窗口内的第二次上升沿 */
    UPPER_REMOTE_AUTO_PD11_WAIT_FIRST_DELAY,  /* 等待第一段延时 */
    UPPER_REMOTE_AUTO_PD11_WAIT_J4310,        /* 等待 J4310 动作时刻 */
    UPPER_REMOTE_AUTO_PD11_WAIT_FINAL_DELAY,  /* 等待最终收尾延时 */
    UPPER_REMOTE_AUTO_PD11_WAIT_FINAL_J4310,  /* PE4 关闭后等待 J4310 收尾 */
    UPPER_REMOTE_AUTO_PD11_WAIT_DOUBLE_RETURN /* 双按流程等待打开 PE4 并回臂 */
} upper_remote_auto_pd11_state_t;

typedef enum
{
    UPPER_REMOTE_PC0_IDLE = 0,
    UPPER_REMOTE_PC0_WAIT_FIRST_PE4_CLOSE,
    UPPER_REMOTE_PC0_WAIT_SECOND_PRESS,
    UPPER_REMOTE_PC0_WAIT_PD8_SECOND,
    UPPER_REMOTE_PC0_WAIT_FINAL_PE4_OPEN,
    UPPER_REMOTE_PC0_WAIT_FINAL_DELAY
} upper_remote_pc0_state_t;

typedef enum
{
    UPPER_REMOTE_PC0_BRANCH_ONE = 0,
    UPPER_REMOTE_PC0_BRANCH_TWO,
    UPPER_REMOTE_PC0_BRANCH_THREE,
    UPPER_REMOTE_PC0_BRANCH_COUNT
} upper_remote_pc0_branch_t;

typedef struct
{
    bool auto_pd13_has_pressed;
    bool auto_pd12_has_pressed;
    bool auto_pd13_branch_two_armed;
    bool auto_pd12_branch_two_armed;
    bool auto_pc0_has_pressed;
    uint8_t auto_pc0_next_branch;
} upper_remote_mode_history_t;

static const motor_cfg_t upper_motor_cfg[UPPER_MOTOR_COUNT] =
{
    [UPPER_MOTOR_ARM_M3508_1] =
    {
        "arm_m3508_1", MOTOR_MODEL_M3508,
        CAN_BUS_ARM_M3508, NODE_ARM_M3508_1,
        UPPER_CONTROL_PERIOD_MS, 0U, true
    },
    [UPPER_MOTOR_ARM_M3508_2] =
    {
        "arm_m3508_2", MOTOR_MODEL_M3508,
        CAN_BUS_ARM_M3508, NODE_ARM_M3508_2,
        UPPER_CONTROL_PERIOD_MS, 0U, true
    },
    [UPPER_MOTOR_ARM_J4310] =
    {
        "arm_j4310", MOTOR_MODEL_J4310,
        CAN_BUS_ARM_J4310, NODE_ARM_J4310,
        UPPER_CONTROL_PERIOD_MS, 0U, true
    },
    [UPPER_MOTOR_GATE_M2006] =
    {
        "gate_m2006", MOTOR_MODEL_M2006,
        CAN_BUS_AUX, NODE_GATE_M2006,
        UPPER_CONTROL_PERIOD_MS, 0U, true
    },
    [UPPER_MOTOR_GRIPPER_M2006] =
    {
        "gripper_m2006", MOTOR_MODEL_M2006,
        CAN_BUS_AUX, NODE_GRIPPER_M2006,
        UPPER_CONTROL_PERIOD_MS, 0U, true
    }
};

static upper_robot_t upper_robot;
static upper_pc_link_t upper_pc_link;
static upper_remote_link_t upper_remote_link;
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
static j4310_position_control_t upper_j4310_position_control;
static bool upper_j4310_auto_return_enabled;
static bool upper_j4310_startup_enable_pending;
static bool upper_j4310_startup_enable_attempted;
static uint32_t upper_j4310_startup_enable_last_tick_ms;
static position_stall_monitor_t upper_j4310_stall_monitor;
static position_stall_monitor_t upper_gate_stall_monitor;
static position_stall_monitor_t upper_gripper_stall_monitor;
static bool upper_gate_stall_rearm_pending;
static bool upper_gripper_stall_rearm_pending;
static bool upper_gripper_stall_protection_enabled;
static bool upper_remote_gate_disable_pending;
static uint8_t upper_remote_previous_primary_key_bits;
static uint8_t upper_remote_previous_key_bits;
static uint8_t upper_remote_previous_switch_bits;
static bool upper_remote_have_switch_state;
static upper_remote_mode_t upper_remote_mode;
static upper_remote_auto_pe4_state_t upper_remote_auto_pe4_state;
static upper_remote_flip_action_t upper_remote_flip_action;
static bool upper_remote_flip_first_stage;
static uint32_t upper_remote_flip_due_tick_ms;
static bool upper_remote_flip_mode;
static upper_remote_auto_pd13_state_t upper_remote_auto_pd13_state;
static uint32_t upper_remote_auto_pd13_due_tick_ms;
static bool upper_remote_auto_pd13_direct_second_pending;
static upper_remote_auto_pd12_state_t upper_remote_auto_pd12_state;
static bool upper_remote_auto_pd12_j4310_pending;
static uint32_t upper_remote_auto_pd12_j4310_due_tick_ms;
static bool upper_remote_auto_pd12_direct_second_pending;
static upper_remote_auto_pd11_state_t upper_remote_auto_pd11_state;
static uint32_t upper_remote_auto_pd11_due_tick_ms;
static upper_remote_pc0_state_t upper_remote_pc0_state;
static upper_remote_pc0_branch_t upper_remote_pc0_active_branch;
static uint32_t upper_remote_pc0_due_tick_ms;
static upper_remote_mode_history_t
    upper_remote_mode_history[UPPER_REMOTE_MODE_COUNT];
static bool upper_remote_pd13_second;
static bool upper_remote_pd12_second;
static bool upper_remote_pd11_second;
static bool upper_remote_pd11_pending;
static uint32_t upper_remote_pd11_due_tick_ms;
static float upper_remote_pd11_pending_j4310_deg;
static bool upper_remote_pd8_second;
static bool upper_remote_pd8_first_pending;
static uint32_t upper_remote_pd8_first_due_tick_ms;
static bool upper_remote_pd9_zero_pending;
static bool upper_remote_pd9_second;
static bool upper_remote_pd10_second;
static volatile uint8_t upper_aux_uart5_pending_count;
static volatile uint8_t upper_aux_output_bits;
static volatile uint8_t upper_aux_update_mask;
static uint8_t upper_aux_uart5_sequence;
static uint8_t upper_aux_uart5_frame[UPPER_AUX_UART_FRAME_SIZE];
volatile uint32_t upper_handshake_ack_sent_count;
volatile uint32_t upper_handshake_ack_busy_count;
volatile uint32_t upper_handshake_ack_fail_count;
volatile uint32_t upper_state_sent_count;
volatile uint32_t upper_state_busy_count;
volatile uint32_t upper_state_fail_count;
volatile uint32_t upper_dji_telemetry_sent_count;
volatile uint32_t upper_dji_telemetry_busy_count;
volatile uint32_t upper_dji_telemetry_fail_count;
volatile uint32_t upper_aux_uart5_sent_count;
volatile uint32_t upper_aux_uart5_fail_count;
__ALIGNED(32) static uint8_t upper_tx_buffer[UPPER_TX_BUFFER_SIZE];

/* 内部函数声明：仅用于按功能分类排列本文件中的函数实现。 */
static upper_pid_cfg_t UpperEntry_ConvertPcPid(
    const upper_pc_pid_cfg_t *source);
static void UpperEntry_ConvertPcTarget(const upper_pc_target_t *source,
                                       upper_target_t *target);
static void UpperEntry_OnPcCmd(const upper_pc_target_t *source,
                               void *user_data);
static void UpperEntry_OnPcEStop(void *user_data);
static void UpperEntry_OnPcAuxControl(uint8_t output_bits,
                                      uint8_t update_mask,
                                      void *user_data);
static uint8_t UpperEntry_Crc8(const uint8_t *data, size_t size);
static void UpperEntry_ProcessAuxUart5(void);
static void UpperEntry_OnUart(comm_uart_channel_t channel,
                              const uint8_t *data,
                              size_t size,
                              void *user_data);
static void UpperEntry_OnCan(uint8_t can_bus,
                             const can_frame_t *frame,
                             void *user_data);
static void UpperEntry_OnPcMotorAction(uint8_t action,
                                       uint8_t can_bus,
                                       uint8_t node_id,
                                       uint8_t value,
                                       void *user_data);
static void UpperEntry_SendHandshakeAck(uint32_t tick_ms);
static void UpperEntry_SendFlashInfo(void);
static void UpperEntry_ProcessPcEvents(uint32_t tick_ms);
static void UpperEntry_SendState(uint32_t tick_ms);
static void UpperEntry_SendDjiTelemetry(uint32_t tick_ms);
static void UpperEntry_CancelJ4310AutoReturn(void);
static bool UpperEntry_ResetJ4310PositionControl(void);
static void UpperEntry_StartJ4310Trajectory(uint32_t tick_ms,
                                             float target_position_rad);
static void UpperEntry_ServiceJ4310StartupEnable(uint32_t tick_ms);
static void UpperEntry_ServiceJ4310Control(uint32_t tick_ms);
static bool UpperEntry_InitStallRecovery(void);
static void UpperEntry_ServiceStallRecovery(uint32_t tick_ms);
static void UpperEntry_ConfigureGripperStallProtection(
    const gripper_target_t *target,
    bool protection_enabled);
static void UpperEntry_ServiceRemoteGateDisable(void);
static float UpperEntry_RemoteDegreesToRadians(float degrees);
static void UpperEntry_ApplyRemoteArm(float m3508_angle_deg,
                                      float j4310_angle_deg,
                                      uint32_t tick_ms);
static void UpperEntry_ApplyRemoteM3508(float angle_deg);
static void UpperEntry_ApplyRemoteJ4310(float angle_deg, uint32_t tick_ms);
static void UpperEntry_ApplyRemoteGate(float angle_deg);
static void UpperEntry_HandleRemotePc1(void);
static void UpperEntry_ResetRemotePc0Sequence(void);
static void UpperEntry_ResetRemotePc0ToFirstBranch(void);
static void UpperEntry_HandleRemotePc0Press(uint32_t tick_ms);
static bool UpperEntry_HandleRemotePc0Pe4(bool pe4_closed,
                                          uint32_t tick_ms);
static void UpperEntry_ServiceRemotePc0Sequence(uint32_t tick_ms);
static void UpperEntry_ApplyRemoteGripper(float angle_deg,
                                           bool stall_protection_enabled);
static void UpperEntry_ProcessCmd(uint32_t tick_ms);
static void UpperEntry_ProcessMotorAction(uint32_t tick_ms);
static void UpperEntry_CheckMotorHealth(uint32_t tick_ms);
static void UpperEntry_ProcessRemote(uint32_t tick_ms);
static void UpperEntry_ResetRemoteModeState(bool clear_history);
static void UpperEntry_OpenRemotePe4Output(void);
static void UpperEntry_CloseRemotePe4Output(void);
static void UpperEntry_HandleRemoteAutomaticPe4(bool pe4_closed,
                                                 uint32_t tick_ms);
static void UpperEntry_ApplyRemoteAutomaticStartOutputs(
    bool first_storage_press);
static bool UpperEntry_IsFirstRemoteAutomaticStoragePress(
    const upper_remote_mode_history_t *history);
static void UpperEntry_StartRemoteFlipAction(
    upper_remote_flip_action_t action,
    uint32_t tick_ms);
static void UpperEntry_HandleRemoteFlipPress(
    upper_remote_flip_action_t action,
    uint32_t tick_ms);
static void UpperEntry_FinishRemoteFlipAction(uint32_t tick_ms);
static void UpperEntry_StartRemoteAutoFlipReset(uint32_t tick_ms);
static void UpperEntry_ServiceRemoteFlipSequence(uint32_t tick_ms);
static void UpperEntry_ResetRemoteAutoPd13Sequence(void);
static void UpperEntry_ResetRemoteAutoPd12Sequence(void);
static void UpperEntry_ResetRemoteAutoPd11Sequence(void);
static void UpperEntry_ResetRemoteAutomaticProgress(void);
static void UpperEntry_ResetRemoteAutomaticSequences(void);
static void UpperEntry_StartRemoteAutoPd13DirectSecond(uint32_t tick_ms);
static void UpperEntry_StartRemoteAutoPd12DirectSecond(uint32_t tick_ms);
static void UpperEntry_HandleRemoteAutoPd13Press(uint32_t tick_ms);
static void UpperEntry_HandleRemoteAutoPd12Press(uint32_t tick_ms);
static void UpperEntry_HandleRemoteAutoPd11Press(uint32_t tick_ms);
static void UpperEntry_ServiceRemoteAutomaticSequences(uint32_t tick_ms);
static void UpperEntry_PrepareRemoteActions(uint8_t action_bits);
static void UpperEntry_ApplyRemotePd13First(uint32_t tick_ms);
static void UpperEntry_ApplyRemotePd13Second(uint32_t tick_ms);
static void UpperEntry_HandleRemotePd13(uint32_t tick_ms);
static void UpperEntry_ApplyRemotePd12First(uint32_t tick_ms);
static void UpperEntry_ApplyRemotePd12Second(uint32_t tick_ms);
static void UpperEntry_HandleRemotePd12(uint32_t tick_ms);
static void UpperEntry_StartRemotePd11(float m3508_angle_deg,
                                       float j4310_angle_deg,
                                       uint32_t tick_ms);
static void UpperEntry_StartRemotePd11First(uint32_t tick_ms);
static void UpperEntry_HandleRemotePd11(uint32_t tick_ms);
static void UpperEntry_ApplyRemotePd8Second(float j4310_angle_deg,
                                             uint32_t tick_ms);
static void UpperEntry_HandleRemotePd8(uint32_t tick_ms);
static void UpperEntry_HandleRemotePd9(void);
static void UpperEntry_HandleRemotePd10(void);
static void UpperEntry_HandleRemotePd9Pd10(uint8_t rising_bits);

/* ==================== UART、CAN 与回调 ==================== */

/* 功能：把 PC 协议中的 PID 配置转换为上层应用格式；参数 source 为源配置；返回转换后的 PID 配置。 */
static upper_pid_cfg_t UpperEntry_ConvertPcPid(
    const upper_pc_pid_cfg_t *source)
{
    return (upper_pid_cfg_t)
    {
        source->kp,
        source->ki,
        source->kd,
        source->integral_limit,
        source->output_limit
    };
}

/* 功能：把 PC 下发的整机目标转换为机器人控制目标；参数 source 为 PC 目标，target 用于接收转换结果。 */
static void UpperEntry_ConvertPcTarget(const upper_pc_target_t *source,
                                       upper_target_t *target)
{
    size_t index;

    (void)memset(target, 0, sizeof(*target));
    target->position_mode = source->position_mode;
    target->arm.enabled = source->arm.enabled;
    target->arm.j4310_commanded = source->arm.j4310_commanded;
    target->arm.m3508_enabled = source->arm.m3508_enabled;
    target->arm.position_mode = source->arm.position_mode;
    target->arm.grip_pos_rad = source->arm.grip_pos_rad;
    target->arm.grip_vel_rad_s = source->arm.grip_vel_rad_s;
    target->arm.grip_kp = source->arm.grip_kp;
    target->arm.grip_kd = source->arm.grip_kd;
    target->arm.grip_torque_nm = source->arm.grip_torque_nm;
    target->arm.grip_torque_limit_nm = source->arm.grip_torque_limit_nm;
    target->arm.pid_update = source->arm.pid_update;
    target->arm.m3508_speed_pid =
        UpperEntry_ConvertPcPid(&source->arm.m3508_speed_pid);
    target->arm.m3508_position_pid =
        UpperEntry_ConvertPcPid(&source->arm.m3508_position_pid);
    for (index = 0U; index < UPPER_ARM_M3508_COUNT; index++)
    {
        target->arm.m3508_vel_rad_s[index] =
            source->arm.m3508_vel_rad_s[index];
        target->arm.m3508_pos_rad[index] =
            source->arm.m3508_pos_rad[index];
    }

    target->gate.enabled = source->gate.enabled;
    target->gate.position_mode = source->gate.position_mode;
    target->gate.m2006_vel_rad_s = source->gate.m2006_vel_rad_s;
    target->gate.m2006_pos_rad = source->gate.m2006_pos_rad;
    target->gate.pid_update = source->gate.pid_update;
    target->gate.m2006_speed_pid =
        UpperEntry_ConvertPcPid(&source->gate.m2006_speed_pid);
    target->gate.m2006_position_pid =
        UpperEntry_ConvertPcPid(&source->gate.m2006_position_pid);

    target->gripper.enabled = source->gripper.enabled;
    target->gripper.position_mode = source->gripper.position_mode;
    target->gripper.m2006_vel_rad_s = source->gripper.m2006_vel_rad_s;
    target->gripper.m2006_pos_rad = source->gripper.m2006_pos_rad;
    target->gripper.pid_update = source->gripper.pid_update;
    target->gripper.m2006_speed_pid =
        UpperEntry_ConvertPcPid(&source->gripper.m2006_speed_pid);
    target->gripper.m2006_position_pid =
        UpperEntry_ConvertPcPid(&source->gripper.m2006_position_pid);
}

/* 功能：接收上位机目标回调并暂存命令；用途：把通信上下文的数据安全移交给控制周期；无返回值表示设置待处理标志。 */
static void UpperEntry_OnPcCmd(const upper_pc_target_t *source,
                               void *user_data)
{
    upper_target_t target;

    (void)user_data;
    UpperEntry_ConvertPcTarget(source, &target);
    if (osMessageQueuePut(upper_cmd_queue, &target, 0U, 0U) != osOK)
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

/* 功能：保存 PC 下发的辅助输出位；用途：由周期任务通过 UART5 转发到抬升 H723。 */
static void UpperEntry_OnPcAuxControl(uint8_t output_bits,
                                      uint8_t update_mask,
                                      void *user_data)
{
    (void)user_data;
    output_bits &= UPPER_AUX_OUTPUT_MASK;
    update_mask &= UPPER_AUX_OUTPUT_MASK;
    if (update_mask == 0U)
    {
        /* Legacy GUI: PB3 was explicit; the other channels were edge-based. */
        update_mask = (upper_aux_output_bits ^ output_bits) |
                      UPPER_AUX_OUTPUT_PE4;
    }
    upper_aux_output_bits =
        (upper_aux_output_bits & (uint8_t)~update_mask) |
        (output_bits & update_mask);
    upper_aux_update_mask |= update_mask;
    upper_aux_uart5_pending_count = UPPER_AUX_UART_REPEAT_COUNT;
}

/* 功能：计算辅助帧 CRC8；用途：保护 UART 桥接数据；多项式为 0x07、初值为 0。 */
static uint8_t UpperEntry_Crc8(const uint8_t *data, size_t size)
{
    uint8_t crc = 0U;
    size_t index;
    uint8_t bit;

    for (index = 0U; index < size; index++)
    {
        crc ^= data[index];
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = ((crc & 0x80U) != 0U) ?
                  (uint8_t)((crc << 1U) ^ 0x07U) :
                  (uint8_t)(crc << 1U);
        }
    }
    return crc;
}

/* 功能：发送待处理的辅助控制帧；用途：状态变化时经 UART5 转发给抬升 H723。 */
static void UpperEntry_ProcessAuxUart5(void)
{
    uint8_t output_bits;
    uint8_t update_mask;

    if (upper_aux_uart5_pending_count == 0U)
    {
        return;
    }
    if (!CommRuntime_AuxUartTxReady())
    {
        return;
    }

    output_bits = upper_aux_output_bits;
    update_mask = upper_aux_update_mask;
    upper_aux_uart5_frame[0] = 0xA5U;
    upper_aux_uart5_frame[1] = 0x5AU;
    upper_aux_uart5_frame[2] = UPPER_AUX_UART_TARGET_RECEIVER;
    upper_aux_uart5_frame[3] = UPPER_AUX_UART_TYPE_CONTROL;
    upper_aux_uart5_frame[4] = UPPER_AUX_UART_PAYLOAD_SIZE;
    upper_aux_uart5_frame[5] = upper_aux_uart5_sequence++;
    upper_aux_uart5_frame[6] =
        output_bits | (uint8_t)(update_mask << UPPER_AUX_UPDATE_SHIFT);
    upper_aux_uart5_frame[7] = UpperEntry_Crc8(
                                   &upper_aux_uart5_frame[2],
                                   UPPER_AUX_UART_FRAME_SIZE - 3U);
    if (CommRuntime_AuxUartTransmit(upper_aux_uart5_frame,
                                    UPPER_AUX_UART_FRAME_SIZE))
    {
        if ((upper_aux_output_bits == output_bits) &&
            (upper_aux_update_mask == update_mask))
        {
            upper_aux_uart5_pending_count--;
            if (upper_aux_uart5_pending_count == 0U)
            {
                upper_aux_update_mask = 0U;
            }
        }
        upper_aux_uart5_sent_count++;
    }
    else
    {
        upper_aux_uart5_fail_count++;
    }
}

/* 功能：处理通信层转交的 UART 数据；用途：仅将选定控制通道数据送入上位机协议；无返回值表示数据已消费或忽略。 */
static void UpperEntry_OnUart(comm_uart_channel_t channel,
                              const uint8_t *data,
                              size_t size,
                              void *user_data)
{
    /* UART4 用于 PC 链路；UART5 传输固定 10 字节的遥控帧。 */
    (void)user_data;
    if (channel == COMM_UART_PC)
    {
        UpperEntry_OnPcData(data, size, CommRuntime_GetTickMs());
    }
    else if (channel == COMM_UART_UART5)
    {
        UpperRemoteLink_Push(&upper_remote_link,
                             data,
                             size,
                             CommRuntime_GetTickMs());
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

/* Flash 诊断独立于电机控制会话。 */
/* 功能：在请求待处理时构造并发送 Flash 信息；用途：响应 PC 对外部 Flash 状态的查询；无返回值表示发送流程已处理。 */
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

/* 功能：处理上位机事件响应；用途：发送握手、维护结果、故障和 Flash 信息。 */
static void UpperEntry_ProcessPcEvents(uint32_t tick_ms)
{
    size_t frame_size;
    upper_motor_fault_t fault;

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
}

/* 周期上报机器人状态和 J4310 反馈；事件帧在本周期中拥有更高优先级。 */
static void UpperEntry_SendState(uint32_t tick_ms)
{
    size_t frame_size;
    upper_j4310_rx_diagnostic_t j4310_rx_diagnostic;
    upper_j4310_tx_diagnostic_t j4310_tx_diagnostic;
    upper_pc_state_t pc_state;
    float j4310_position_rad;
    bool j4310_position_valid;
    bool j4310_diagnostic_valid;
    bool j4310_tx_diagnostic_valid;

    if (!UpperPcLink_IsSessionActive(&upper_pc_link, tick_ms) ||
        ((tick_ms - upper_state_last_sent_tick_ms) < UPPER_STATE_PERIOD_MS))
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
    (void)memset(&pc_state, 0, sizeof(pc_state));
    pc_state.robot_state = (uint8_t)upper_robot.state;
    pc_state.motor_sent_count = upper_robot.motor_manager.sent_count;
    pc_state.motor_send_fail_count =
        upper_robot.motor_manager.send_fail_count;
    pc_state.motor_protocol_block_count =
        upper_robot.motor_manager.protocol_block_count;
    pc_state.j4310_position_valid = j4310_position_valid;
    pc_state.j4310_position_rad = j4310_position_valid ?
                                  j4310_position_rad : 0.0f;
    pc_state.j4310_bus_rx_frames =
        comm_fdcan_rx_count[CAN_BUS_ARM_J4310 - 1U];
    pc_state.j4310_rx_valid = j4310_diagnostic_valid;
    if (j4310_diagnostic_valid)
    {
        pc_state.j4310_rx.accepted_frames =
            j4310_rx_diagnostic.accepted_frames;
        pc_state.j4310_rx.rejected_format_frames =
            j4310_rx_diagnostic.rejected_format_frames;
        pc_state.j4310_rx.rejected_master_id_frames =
            j4310_rx_diagnostic.rejected_master_id_frames;
        pc_state.j4310_rx.rejected_feedback_id_frames =
            j4310_rx_diagnostic.rejected_feedback_id_frames;
        pc_state.j4310_rx.last_can_id = j4310_rx_diagnostic.last_can_id;
        pc_state.j4310_rx.last_dlc = j4310_rx_diagnostic.last_dlc;
        pc_state.j4310_rx.last_data0 = j4310_rx_diagnostic.last_data0;
        pc_state.j4310_rx.last_result = j4310_rx_diagnostic.last_result;
    }
    pc_state.j4310_tx_valid = j4310_tx_diagnostic_valid;
    if (j4310_tx_diagnostic_valid)
    {
        pc_state.j4310_tx.attempted_frames =
            j4310_tx_diagnostic.attempted_frames;
        pc_state.j4310_tx.queued_frames =
            j4310_tx_diagnostic.queued_frames;
        pc_state.j4310_tx.failed_frames =
            j4310_tx_diagnostic.failed_frames;
        pc_state.j4310_tx.enable_frames =
            j4310_tx_diagnostic.enable_frames;
        pc_state.j4310_tx.mit_frames = j4310_tx_diagnostic.mit_frames;
        pc_state.j4310_tx.disable_frames =
            j4310_tx_diagnostic.disable_frames;
        pc_state.j4310_tx.last_can_id = j4310_tx_diagnostic.last_can_id;
        pc_state.j4310_tx.last_dlc = j4310_tx_diagnostic.last_dlc;
        pc_state.j4310_tx.last_data7 = j4310_tx_diagnostic.last_data7;
        pc_state.j4310_tx.enable_confirmed =
            j4310_tx_diagnostic.enable_confirmed;
        pc_state.j4310_tx.feedback_state =
            j4310_tx_diagnostic.feedback_state;
    }
    pc_state.j4310_auto_return.available = true;
    pc_state.j4310_auto_return.enabled = upper_j4310_auto_return_enabled;
    pc_state.j4310_auto_return.active =
        J4310AutoReturn_IsActive(&upper_j4310_auto_return);
    pc_state.j4310_auto_return.stage =
        (uint8_t)upper_j4310_auto_return.stage;
    frame_size = UpperPcLink_BuildState(&upper_pc_link,
                                        &pc_state,
                                        tick_ms,
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

/* 分时上报各 DJI 电机的实时位置、零点状态和反馈新鲜度。 */
static void UpperEntry_SendDjiTelemetry(uint32_t tick_ms)
{
    upper_dji_diagnostic_t diagnostics[UPPER_PC_DJI_DIAGNOSTIC_COUNT];
    uint32_t fdcan_rx_counts[UPPER_PC_FDCAN_COUNT];
    upper_pc_dji_telemetry_t telemetry;
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
    telemetry.model =
        (uint8_t)diagnostics[upper_dji_telemetry_next_index].model;
    telemetry.can_bus =
        diagnostics[upper_dji_telemetry_next_index].can_bus;
    telemetry.node_id =
        diagnostics[upper_dji_telemetry_next_index].node_id;
    telemetry.feedback_received =
        diagnostics[upper_dji_telemetry_next_index].feedback_received;
    telemetry.zero_valid =
        diagnostics[upper_dji_telemetry_next_index].zero_valid;
    telemetry.feedback_fresh =
        diagnostics[upper_dji_telemetry_next_index].feedback_fresh;
    telemetry.rotor_position_rad =
        diagnostics[upper_dji_telemetry_next_index].rotor_position_rad;
    telemetry.zero_rotor_position_rad =
        diagnostics[upper_dji_telemetry_next_index].zero_rotor_position_rad;
    telemetry.relative_output_position_rad =
        diagnostics[upper_dji_telemetry_next_index].relative_output_position_rad;
    frame_size = UpperPcLink_BuildDjiTelemetry(
                     &upper_pc_link,
                     &telemetry,
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
/* ==================== 底层电机控制 ==================== */


/* 功能：取消 J4310 自动回零并释放其控制权；用途：切换到遥控或 PC 主动控制；无返回值表示回零状态已清除。 */
static void UpperEntry_CancelJ4310AutoReturn(void)
{
    J4310AutoReturn_Cancel(&upper_j4310_auto_return);
    (void)MotorManager_ClearOverride(&upper_robot.motor_manager,
                                     UPPER_MOTOR_ARM_J4310);
}

/* 功能：按上层配置重新初始化 J4310 位置控制器；用途：恢复轨迹与重力补偿的初始状态；返回 true 表示初始化成功。 */
static bool UpperEntry_ResetJ4310PositionControl(void)
{
    return J4310PositionControl_Init(
        &upper_j4310_position_control,
        UPPER_J4310_TRAJECTORY_MAX_VEL_RAD_S,
        UPPER_J4310_TRAJECTORY_MAX_ACCEL_RAD_S2,
        UPPER_J4310_GRAVITY_MODEL_LIMIT_NM,
        UPPER_J4310_GRAVITY_LEARNING_RATE,
        UPPER_J4310_GRAVITY_COMPENSATION_GAIN,
        UPPER_J4310_GRAVITY_DISABLE_HALF_WIDTH_RAD,
        UPPER_J4310_GRAVITY_SETTLE_ERROR_RAD,
        UPPER_J4310_GRAVITY_SETTLE_FEEDBACK_COUNT,
        UPPER_J4310_GRAVITY_TORQUE_RATE_NM_S);
}

/* 功能：为 J4310 启动到指定角度的位置轨迹；用途：将离散目标转换为平滑运动；无返回值表示启动请求已处理。 */
static void UpperEntry_StartJ4310Trajectory(uint32_t tick_ms,
                                             float target_position_rad)
{
    upper_j4310_feedback_t feedback;
    float start_position_rad;

    if (UpperMotorPort_GetJ4310Feedback(CAN_BUS_ARM_J4310,
                                        NODE_ARM_J4310,
                                        &feedback))
    {
        start_position_rad = feedback.position_rad;
    }
    else if (upper_robot.target.arm.enabled)
    {
        start_position_rad = upper_robot.target.arm.grip_pos_rad;
    }
    else
    {
        start_position_rad = target_position_rad;
    }
    if (!J4310PositionControl_Start(&upper_j4310_position_control,
                                     tick_ms,
                                     start_position_rad,
                                     target_position_rad))
    {
        J4310PositionControl_Hold(&upper_j4310_position_control,
                                  target_position_rad);
    }
}

/* 功能：上电使能 J4310 并锁存首个有效角度；用途：确认使能后直接保持当时位置。 */
static void UpperEntry_ServiceJ4310StartupEnable(uint32_t tick_ms)
{
    upper_j4310_tx_diagnostic_t diagnostic;
    upper_j4310_feedback_t feedback;

    if (!upper_j4310_startup_enable_pending)
    {
        return;
    }
    if (UpperMotorPort_GetJ4310TxDiagnostic(CAN_BUS_ARM_J4310,
                                            NODE_ARM_J4310,
                                            &diagnostic) &&
        diagnostic.enable_confirmed)
    {
        if (UpperMotorPort_GetJ4310Feedback(CAN_BUS_ARM_J4310,
                                            NODE_ARM_J4310,
                                            &feedback))
        {
            upper_target_t target = upper_robot.target;

            target.arm.enabled = true;
            target.arm.j4310_commanded = false;
            target.arm.position_mode = true;
            target.arm.grip_pos_rad = feedback.position_rad;
            target.arm.grip_vel_rad_s = 0.0f;
            target.arm.grip_kp = UPPER_J4310_POSITION_KP;
            target.arm.grip_kd = UPPER_J4310_POSITION_KD;
            target.arm.grip_torque_nm = 0.0f;
            target.arm.grip_torque_limit_nm =
                UPPER_J4310_TORQUE_MAP_MAX_NM;
            J4310PositionControl_Hold(&upper_j4310_position_control,
                                      feedback.position_rad);
            UpperRobot_SetTarget(&upper_robot, &target);
            UpperRobot_Start(&upper_robot);
            upper_j4310_startup_enable_pending = false;
        }
        return;
    }
    if (upper_j4310_startup_enable_attempted &&
        ((tick_ms - upper_j4310_startup_enable_last_tick_ms) <
         UPPER_J4310_STARTUP_ENABLE_RETRY_MS))
    {
        return;
    }

    upper_j4310_startup_enable_attempted = true;
    upper_j4310_startup_enable_last_tick_ms = tick_ms;
    (void)UpperMotorPort_EnableJ4310(CAN_BUS_ARM_J4310,
                                     NODE_ARM_J4310);
}

/* 功能：执行 J4310 位置轨迹和自动回零的周期服务；用途：更新目标并合成关节控制命令；无返回值表示本周期控制已完成。 */
static void UpperEntry_ServiceJ4310Control(uint32_t tick_ms)
{
    const size_t index = UPPER_MOTOR_ARM_J4310;
    upper_j4310_feedback_t feedback;
    bool feedback_fresh;
    motor_cmd_t command;

    feedback_fresh = UpperMotorPort_GetJ4310Feedback(
                         CAN_BUS_ARM_J4310,
                         NODE_ARM_J4310,
                         &feedback);
    J4310AutoReturn_Update(
        &upper_j4310_auto_return,
        tick_ms,
        feedback_fresh,
        feedback_fresh ? feedback.position_rad : 0.0f,
        feedback_fresh ? feedback.velocity_rad_s : 0.0f,
        upper_robot.state == ROBOT_RUN);
    if ((upper_robot.state != ROBOT_RUN) ||
        !upper_robot.target.arm.enabled)
    {
        (void)MotorManager_ClearOverride(&upper_robot.motor_manager,
                                         index);
        return;
    }

    (void)memset(&command, 0, sizeof(command));
    command.mode = MOTOR_CMD_MIT;
    if (J4310AutoReturn_IsActive(&upper_j4310_auto_return))
    {
        J4310PositionControl_CancelTrajectory(
            &upper_j4310_position_control);
        command.pos_rad = upper_j4310_auto_return.target_position_rad;
        command.vel_rad_s = 0.0f;
        command.kp = UPPER_J4310_POSITION_KP;
        command.kd = UPPER_J4310_POSITION_KD;
    }
    else
    {
        J4310PositionControl_Sample(&upper_j4310_position_control,
                                    tick_ms,
                                    &command.pos_rad,
                                    NULL);
        command.vel_rad_s = 0.0f;
        command.kp = upper_robot.target.arm.grip_kp;
        command.kd = upper_robot.target.arm.grip_kd;
    }
    command.torque_nm = J4310PositionControl_ComposeTorque(
        &upper_j4310_position_control,
        feedback_fresh,
        feedback_fresh ? feedback.updated_at_ms : 0U,
        feedback_fresh ? feedback.position_rad : 0.0f,
        feedback_fresh ? feedback.velocity_rad_s : 0.0f,
        feedback_fresh ? feedback.torque_nm : 0.0f,
        command.pos_rad,
        command.vel_rad_s,
        upper_robot.target.arm.grip_torque_nm,
        upper_robot.target.arm.grip_torque_limit_nm);
    (void)MotorManager_SetOverride(&upper_robot.motor_manager,
                                   index,
                                   &command);
    UpperRobot_Start(&upper_robot);
}

/* 功能：把遥控协议中的角度从度转换为弧度；用途：统一应用层内部角度单位；返回值表示弧度值。 */
static float UpperEntry_RemoteDegreesToRadians(float degrees)
{
    return degrees * UPPER_REMOTE_ANGLE_DEG_TO_RAD;
}

/* 为 J4310、闸门和夹爪设置特意偏保守的堵转阈值。 */
static bool UpperEntry_InitStallRecovery(void)
{
    position_stall_monitor_cfg_t cfg;

    cfg.minimum_error_rad = UpperEntry_RemoteDegreesToRadians(
        UPPER_J4310_STALL_MIN_ERROR_DEG);
    cfg.maximum_velocity_rad_s = UpperEntry_RemoteDegreesToRadians(
        UPPER_J4310_STALL_MAX_VELOCITY_DEG_S);
    cfg.minimum_effort = UPPER_J4310_STALL_MIN_TORQUE_NM;
    cfg.arming_grace_ms = UPPER_STALL_ARMING_GRACE_MS;
    cfg.stall_duration_ms = UPPER_STALL_CONFIRM_MS;
    if (!PositionStallMonitor_Init(&upper_j4310_stall_monitor, &cfg))
    {
        return false;
    }

    cfg.minimum_error_rad = UpperEntry_RemoteDegreesToRadians(
        UPPER_GATE_STALL_MIN_ERROR_DEG);
    cfg.maximum_velocity_rad_s = UpperEntry_RemoteDegreesToRadians(
        UPPER_GATE_STALL_MAX_VELOCITY_DEG_S);
    cfg.minimum_effort = UPPER_GATE_STALL_MIN_CURRENT_A;
    if (!PositionStallMonitor_Init(&upper_gate_stall_monitor, &cfg))
    {
        return false;
    }

    cfg.minimum_error_rad = UpperEntry_RemoteDegreesToRadians(
        UPPER_GRIPPER_STALL_MIN_ERROR_DEG);
    cfg.maximum_velocity_rad_s = UpperEntry_RemoteDegreesToRadians(
        UPPER_GRIPPER_STALL_MAX_VELOCITY_DEG_S);
    cfg.minimum_effort = UPPER_GRIPPER_STALL_MIN_CURRENT_A;
    if (!PositionStallMonitor_Init(&upper_gripper_stall_monitor, &cfg))
    {
        return false;
    }
    upper_gate_stall_rearm_pending = false;
    upper_gripper_stall_rearm_pending = false;
    upper_gripper_stall_protection_enabled = true;
    return true;
}

static void UpperEntry_ConfigureGripperStallProtection(
    const gripper_target_t *target,
    bool protection_enabled)
{
    if ((target == NULL) || !target->enabled || !target->position_mode)
    {
        upper_gripper_stall_protection_enabled = true;
        upper_gripper_stall_rearm_pending = false;
        PositionStallMonitor_Disarm(&upper_gripper_stall_monitor);
        return;
    }

    upper_gripper_stall_protection_enabled = protection_enabled;
    upper_gripper_stall_rearm_pending =
        upper_gripper_stall_protection_enabled;
    if (!upper_gripper_stall_protection_enabled)
    {
        PositionStallMonitor_Disarm(&upper_gripper_stall_monitor);
    }
}

/* 中止过期的遥控流程阶段，并仅下发一次请求的恢复位置。 */
static void UpperEntry_ServiceStallRecovery(uint32_t tick_ms)
{
    upper_j4310_feedback_t j4310_feedback;
    upper_m2006_feedback_t gate_feedback;
    upper_m2006_feedback_t gripper_feedback;
    upper_target_t target;
    bool gate_disable_pending;
    bool feedback_valid;

    if (upper_gate_stall_rearm_pending)
    {
        PositionStallMonitor_Arm(&upper_gate_stall_monitor,
            upper_robot.target.gate.m2006_pos_rad, tick_ms);
        upper_gate_stall_rearm_pending = false;
    }
    if (upper_gripper_stall_rearm_pending &&
        upper_gripper_stall_protection_enabled)
    {
        PositionStallMonitor_Arm(&upper_gripper_stall_monitor,
            upper_robot.target.gripper.m2006_pos_rad, tick_ms);
        upper_gripper_stall_rearm_pending = false;
    }

    feedback_valid = UpperMotorPort_GetJ4310Feedback(
        CAN_BUS_ARM_J4310, NODE_ARM_J4310, &j4310_feedback);
    if (PositionStallMonitor_Update(
            &upper_j4310_stall_monitor,
            tick_ms,
            feedback_valid,
            feedback_valid ? j4310_feedback.position_rad : 0.0f,
            feedback_valid ? j4310_feedback.velocity_rad_s : 0.0f,
            feedback_valid ? j4310_feedback.torque_nm : 0.0f))
    {
        UpperEntry_ResetRemoteModeState(false);
        UpperEntry_ApplyRemoteJ4310(UPPER_J4310_STALL_RECOVERY_DEG,
                                    tick_ms);
        PositionStallMonitor_Disarm(&upper_j4310_stall_monitor);
    }

    feedback_valid = UpperMotorPort_GetM2006Feedback(
        CAN_BUS_AUX, NODE_GATE_M2006, &gate_feedback);
    if (PositionStallMonitor_Update(
            &upper_gate_stall_monitor,
            tick_ms,
            feedback_valid,
            feedback_valid ? gate_feedback.position_rad : 0.0f,
            feedback_valid ? gate_feedback.velocity_rad_s : 0.0f,
            feedback_valid ? gate_feedback.current_a : 0.0f))
    {
        gate_disable_pending = upper_remote_gate_disable_pending;
        UpperEntry_ResetRemoteModeState(false);
        UpperEntry_ApplyRemoteGate(UPPER_GATE_STALL_RECOVERY_DEG);
        upper_remote_gate_disable_pending = gate_disable_pending;
        upper_gate_stall_rearm_pending = false;
        PositionStallMonitor_Disarm(&upper_gate_stall_monitor);
        upper_remote_pd9_zero_pending = false;
        upper_remote_pd9_second = false;
    }

    feedback_valid = UpperMotorPort_GetM2006Feedback(
        CAN_BUS_AUX, NODE_GRIPPER_M2006, &gripper_feedback);
    if (upper_gripper_stall_protection_enabled &&
        PositionStallMonitor_Update(
            &upper_gripper_stall_monitor,
            tick_ms,
            feedback_valid,
            feedback_valid ? gripper_feedback.position_rad : 0.0f,
            feedback_valid ? gripper_feedback.velocity_rad_s : 0.0f,
            feedback_valid ? gripper_feedback.current_a : 0.0f))
    {
        target = upper_robot.target;
        target.gripper.enabled = false;
        target.gripper.pid_update = false;
        upper_gripper_stall_rearm_pending = false;
        UpperRobot_SetTarget(&upper_robot, &target);
    }
}

/* 实测位置进入 79 至 81 度范围后立即禁用闸门。 */
static void UpperEntry_ServiceRemoteGateDisable(void)
{
    upper_m2006_feedback_t feedback;
    upper_target_t target;
    float minimum_position_rad;
    float maximum_position_rad;

    if (!upper_remote_gate_disable_pending ||
        !UpperMotorPort_GetM2006Feedback(CAN_BUS_AUX,
                                         NODE_GATE_M2006,
                                         &feedback))
    {
        return;
    }

    minimum_position_rad = UpperEntry_RemoteDegreesToRadians(
        UPPER_REMOTE_PC1_GATE_DISABLE_MIN_DEG);
    maximum_position_rad = UpperEntry_RemoteDegreesToRadians(
        UPPER_REMOTE_PC1_GATE_DISABLE_MAX_DEG);
    if ((feedback.position_rad < minimum_position_rad) ||
        (feedback.position_rad > maximum_position_rad))
    {
        return;
    }

    target = upper_robot.target;
    target.gate.enabled = false;
    target.gate.pid_update = false;
    upper_remote_gate_disable_pending = false;
    upper_gate_stall_rearm_pending = false;
    PositionStallMonitor_Disarm(&upper_gate_stall_monitor);
    UpperRobot_SetTarget(&upper_robot, &target);
}

/* 应用完整的机械臂目标，使两台 M3508 与 J4310 同步运动。 */
/* 功能：应用遥控器给出的机械臂双电机角度目标；用途：同步更新 M3508 与 J4310 关节命令；无返回值表示目标已提交。 */
static void UpperEntry_ApplyRemoteArm(float m3508_angle_deg,
                                      float j4310_angle_deg,
                                      uint32_t tick_ms)
{
    upper_target_t target;
    float m3508_angle_rad;

    target = upper_robot.target;
    m3508_angle_rad = UpperEntry_RemoteDegreesToRadians(m3508_angle_deg);
    target.position_mode = true;
    target.arm.enabled = true;
    target.arm.j4310_commanded = true;
    target.arm.m3508_enabled = true;
    target.arm.position_mode = true;
    target.arm.grip_pos_rad =
        UpperEntry_RemoteDegreesToRadians(j4310_angle_deg);
    target.arm.grip_vel_rad_s = 0.0f;
    target.arm.grip_kp = UPPER_J4310_POSITION_KP;
    target.arm.grip_kd = UPPER_J4310_POSITION_KD;
    target.arm.grip_torque_nm = 0.0f;
    target.arm.grip_torque_limit_nm = UPPER_J4310_TORQUE_MAP_MAX_NM;
    target.arm.pid_update = false;
    target.arm.m3508_pos_rad[0] = m3508_angle_rad;
    target.arm.m3508_pos_rad[1] = m3508_angle_rad;

    upper_j4310_startup_enable_pending = false;
    UpperEntry_CancelJ4310AutoReturn();
    UpperEntry_StartJ4310Trajectory(tick_ms,
                                     target.arm.grip_pos_rad);
    PositionStallMonitor_Arm(&upper_j4310_stall_monitor,
                             target.arm.grip_pos_rad,
                             tick_ms);
    UpperRobot_SetTarget(&upper_robot, &target);
    UpperRobot_Start(&upper_robot);
}

/* 保持当前 J4310 目标，仅运动两台 M3508。 */
/* 功能：应用遥控器给出的机械臂 M3508 角度目标；用途：单独控制机械臂前级关节；无返回值表示目标已提交。 */
static void UpperEntry_ApplyRemoteM3508(float angle_deg)
{
    upper_target_t target;
    float angle_rad;

    target = upper_robot.target;
    angle_rad = UpperEntry_RemoteDegreesToRadians(angle_deg);
    target.position_mode = true;
    target.arm.enabled = true;
    target.arm.j4310_commanded = false;
    target.arm.m3508_enabled = true;
    target.arm.position_mode = true;
    target.arm.pid_update = false;
    target.arm.m3508_pos_rad[0] = angle_rad;
    target.arm.m3508_pos_rad[1] = angle_rad;

    upper_j4310_startup_enable_pending = false;
    UpperEntry_CancelJ4310AutoReturn();
    UpperRobot_SetTarget(&upper_robot, &target);
    UpperRobot_Start(&upper_robot);
}

/* 保持两台 M3508 目标不变，仅运动 J4310。 */
/* 功能：应用遥控器给出的 J4310 角度目标；用途：启动末端关节的平滑位置轨迹；无返回值表示目标已提交。 */
static void UpperEntry_ApplyRemoteJ4310(float angle_deg, uint32_t tick_ms)
{
    upper_target_t target;

    target = upper_robot.target;
    target.position_mode = true;
    target.arm.enabled = true;
    target.arm.j4310_commanded = true;
    target.arm.position_mode = true;
    target.arm.grip_pos_rad = UpperEntry_RemoteDegreesToRadians(angle_deg);
    target.arm.grip_vel_rad_s = 0.0f;
    target.arm.grip_kp = UPPER_J4310_POSITION_KP;
    target.arm.grip_kd = UPPER_J4310_POSITION_KD;
    target.arm.grip_torque_nm = 0.0f;
    target.arm.grip_torque_limit_nm = UPPER_J4310_TORQUE_MAP_MAX_NM;

    upper_j4310_startup_enable_pending = false;
    UpperEntry_CancelJ4310AutoReturn();
    UpperEntry_StartJ4310Trajectory(tick_ms, target.arm.grip_pos_rad);
    PositionStallMonitor_Arm(&upper_j4310_stall_monitor,
                             target.arm.grip_pos_rad,
                             tick_ms);
    UpperRobot_SetTarget(&upper_robot, &target);
    UpperRobot_Start(&upper_robot);
}

/* 功能：应用遥控器给出的闸门角度目标；用途：更新闸门位置和使能状态；无返回值表示目标已提交。 */
static void UpperEntry_ApplyRemoteGate(float angle_deg)
{
    upper_target_t target;

    upper_remote_gate_disable_pending = false;
    target = upper_robot.target;
    target.position_mode = true;
    target.gate.enabled = true;
    target.gate.position_mode = true;
    target.gate.pid_update = false;
    target.gate.m2006_pos_rad =
        UpperEntry_RemoteDegreesToRadians(angle_deg);
    upper_gate_stall_rearm_pending = true;
    UpperRobot_SetTarget(&upper_robot, &target);
    UpperRobot_Start(&upper_robot);
}

/* PC1 将闸门移动到 80 度，随后反馈服务将其禁用。 */
static void UpperEntry_HandleRemotePc1(void)
{
    UpperEntry_ApplyRemoteGate(UPPER_REMOTE_PC1_GATE_DEG);
    upper_remote_gate_disable_pending = true;
}

static void UpperEntry_ResetRemotePc0Sequence(void)
{
    upper_remote_pc0_state = UPPER_REMOTE_PC0_IDLE;
    upper_remote_pc0_active_branch = UPPER_REMOTE_PC0_BRANCH_ONE;
    upper_remote_pc0_due_tick_ms = 0U;
}

static void UpperEntry_ResetRemotePc0ToFirstBranch(void)
{
    UpperEntry_ResetRemotePc0Sequence();
    upper_remote_mode_history[upper_remote_mode].auto_pc0_next_branch =
        (uint8_t)UPPER_REMOTE_PC0_BRANCH_ONE;
}

static float UpperEntry_RemotePc0BranchM3508Deg(
    upper_remote_pc0_branch_t branch)
{
    if (branch == UPPER_REMOTE_PC0_BRANCH_ONE)
    {
        return UPPER_REMOTE_PD13_FIRST_M3508_DEG;
    }
    if (branch == UPPER_REMOTE_PC0_BRANCH_TWO)
    {
        return UPPER_REMOTE_PC0_SECOND_BRANCH_M3508_DEG;
    }
    return UPPER_REMOTE_PC0_THIRD_BRANCH_M3508_DEG;
}

static void UpperEntry_AdvanceRemotePc0Branch(void)
{
    upper_remote_mode_history_t *history;

    history = &upper_remote_mode_history[upper_remote_mode];
    history->auto_pc0_next_branch =
        (uint8_t)(((uint8_t)upper_remote_pc0_active_branch + 1U) %
                  (uint8_t)UPPER_REMOTE_PC0_BRANCH_COUNT);
}

static void UpperEntry_StartRemotePc0Branch(uint32_t tick_ms)
{
    upper_remote_mode_history_t *history;
    uint8_t next_branch;
    bool first_storage_press;

    history = &upper_remote_mode_history[upper_remote_mode];
    next_branch = history->auto_pc0_next_branch;
    if (next_branch >= (uint8_t)UPPER_REMOTE_PC0_BRANCH_COUNT)
    {
        next_branch = (uint8_t)UPPER_REMOTE_PC0_BRANCH_ONE;
        history->auto_pc0_next_branch = next_branch;
    }
    upper_remote_pc0_active_branch =
        (upper_remote_pc0_branch_t)next_branch;

    upper_remote_pd11_pending = false;
    upper_remote_pd8_first_pending = false;
    upper_remote_auto_pe4_state = UPPER_REMOTE_AUTO_PE4_IDLE;
    UpperEntry_ResetRemoteAutomaticProgress();
    upper_remote_gate_disable_pending = false;
    upper_remote_pc0_due_tick_ms = 0U;

    first_storage_press =
        UpperEntry_IsFirstRemoteAutomaticStoragePress(history);
    history->auto_pc0_has_pressed = true;
    UpperEntry_ApplyRemoteArm(
        UpperEntry_RemotePc0BranchM3508Deg(upper_remote_pc0_active_branch),
        UPPER_REMOTE_PC0_FIRST_J4310_DEG,
        tick_ms);

    UpperEntry_ApplyRemoteAutomaticStartOutputs(first_storage_press);

    upper_remote_pc0_state = UPPER_REMOTE_PC0_WAIT_FIRST_PE4_CLOSE;
}

/* PC0 在三个自动分支间循环，并独立于普通 PD9 的单双次状态。 */
static void UpperEntry_HandleRemotePc0Press(uint32_t tick_ms)
{
    if (upper_remote_pc0_state == UPPER_REMOTE_PC0_IDLE)
    {
        UpperEntry_StartRemotePc0Branch(tick_ms);
    }
    else if (upper_remote_pc0_state ==
             UPPER_REMOTE_PC0_WAIT_FIRST_PE4_CLOSE)
    {
        UpperEntry_AdvanceRemotePc0Branch();
        UpperEntry_StartRemotePc0Branch(tick_ms);
    }
    else if (upper_remote_pc0_state ==
             UPPER_REMOTE_PC0_WAIT_SECOND_PRESS)
    {
        upper_remote_pd11_pending = false;
        upper_remote_pd8_first_pending = false;
        UpperEntry_ApplyRemoteGate(UPPER_REMOTE_PC0_GATE_FIRST_DEG);
        upper_remote_pc0_due_tick_ms =
            tick_ms + UPPER_REMOTE_PC0_PD8_DELAY_MS;
        upper_remote_pc0_state = UPPER_REMOTE_PC0_WAIT_PD8_SECOND;
    }
}

/* 返回 true 表示 PC0 正在等待 PE4，本次边沿不能再被其他流程消费。 */
static bool UpperEntry_HandleRemotePc0Pe4(bool pe4_closed,
                                          uint32_t tick_ms)
{
    if (upper_remote_pc0_state == UPPER_REMOTE_PC0_IDLE)
    {
        return false;
    }

    if ((upper_remote_pc0_state ==
         UPPER_REMOTE_PC0_WAIT_FIRST_PE4_CLOSE) && pe4_closed)
    {
        upper_remote_pd11_pending = false;
        upper_remote_pd8_first_pending = false;
        UpperEntry_ApplyRemoteArm(UPPER_REMOTE_PC0_CLOSE_M3508_DEG,
                                  UPPER_REMOTE_PC0_CLOSE_J4310_DEG,
                                  tick_ms);
        upper_remote_pc0_state = UPPER_REMOTE_PC0_WAIT_SECOND_PRESS;
    }
    else if ((upper_remote_pc0_state ==
              UPPER_REMOTE_PC0_WAIT_FINAL_PE4_OPEN) && !pe4_closed)
    {
        upper_remote_pd11_pending = false;
        upper_remote_pd8_first_pending = false;
        /* 打开 PE4 后立即执行 PC0 收尾动作。 */
        upper_remote_pc0_due_tick_ms =
            tick_ms + UPPER_REMOTE_PC0_FINAL_DELAY_MS;
        upper_remote_pc0_state = UPPER_REMOTE_PC0_WAIT_FINAL_DELAY;
    }

    return true;
}

static void UpperEntry_ServiceRemotePc0Sequence(uint32_t tick_ms)
{
    if ((upper_remote_pc0_state == UPPER_REMOTE_PC0_WAIT_PD8_SECOND) &&
        ((int32_t)(tick_ms - upper_remote_pc0_due_tick_ms) >= 0))
    {
        UpperEntry_ApplyRemotePd8Second(
            UPPER_REMOTE_PC0_SECOND_J4310_DEG, tick_ms);
        upper_remote_pc0_due_tick_ms = 0U;
        upper_remote_pc0_state = UPPER_REMOTE_PC0_WAIT_FINAL_PE4_OPEN;
    }
    else if ((upper_remote_pc0_state == UPPER_REMOTE_PC0_WAIT_FINAL_DELAY) &&
             ((int32_t)(tick_ms - upper_remote_pc0_due_tick_ms) >= 0))
    {
        UpperEntry_ApplyRemoteGate(UPPER_REMOTE_PC0_GATE_FINAL_DEG);
        UpperEntry_ApplyRemoteArm(UPPER_REMOTE_PC0_FINAL_M3508_DEG,
                                  UPPER_REMOTE_PC0_FINAL_J4310_DEG,
                                  tick_ms);
        /* PC0 完成后，普通 PD9 的下一次动作固定从 180 度开始。 */
        upper_remote_pd9_zero_pending = false;
        upper_remote_pd9_second = false;
        UpperEntry_AdvanceRemotePc0Branch();
        UpperEntry_ResetRemotePc0Sequence();
    }
}

/* 功能：应用遥控器给出的夹爪角度目标；用途：更新夹爪位置和使能状态；无返回值表示目标已提交。 */
static void UpperEntry_ApplyRemoteGripper(float angle_deg,
                                           bool stall_protection_enabled)
{
    upper_target_t target;

    target = upper_robot.target;
    target.position_mode = true;
    target.gripper.enabled = true;
    target.gripper.position_mode = true;
    target.gripper.pid_update = false;
    target.gripper.m2006_pos_rad =
        UpperEntry_RemoteDegreesToRadians(
            angle_deg * UPPER_REMOTE_GRIPPER_MOTOR_DEG_PER_OUTPUT_DEG);
    UpperEntry_ConfigureGripperStallProtection(
        &target.gripper, stall_protection_enabled);
    UpperRobot_SetTarget(&upper_robot, &target);
    UpperRobot_Start(&upper_robot);
}

/* 功能：处理待执行的上位机整机目标；用途：更新机器人目标并按需启动运行；无返回值表示待处理标志被消费。 */
static void UpperEntry_ProcessCmd(uint32_t tick_ms)
{
    upper_target_t target;
    upper_target_t next_target;
    bool received;
    bool j4310_commanded;

    received = false;
    j4310_commanded = false;
    while (osMessageQueueGet(upper_cmd_queue,
                             &next_target,
                             NULL,
                             0U) == osOK)
    {
        const arm_target_t *previous_arm;

        previous_arm = received ? &target.arm : &upper_robot.target.arm;
        if (!next_target.arm.j4310_commanded)
        {
            next_target.arm.enabled = previous_arm->enabled;
            next_target.arm.grip_pos_rad = previous_arm->grip_pos_rad;
            next_target.arm.grip_vel_rad_s = previous_arm->grip_vel_rad_s;
            next_target.arm.grip_kp = previous_arm->grip_kp;
            next_target.arm.grip_kd = previous_arm->grip_kd;
            next_target.arm.grip_torque_nm = previous_arm->grip_torque_nm;
            next_target.arm.grip_torque_limit_nm =
                previous_arm->grip_torque_limit_nm;
        }
        target = next_target;
        received = true;
        j4310_commanded = j4310_commanded ||
                          next_target.arm.j4310_commanded;
    }

    if (received)
    {
        upper_remote_gate_disable_pending = false;
        UpperEntry_ResetRemotePc0Sequence();
        if (j4310_commanded)
        {
            upper_j4310_startup_enable_pending = false;
            UpperEntry_CancelJ4310AutoReturn();
            if (target.arm.enabled)
            {
                UpperEntry_StartJ4310Trajectory(
                    tick_ms, target.arm.grip_pos_rad);
                if (target.arm.position_mode)
                {
                    PositionStallMonitor_Arm(&upper_j4310_stall_monitor,
                                             target.arm.grip_pos_rad,
                                             tick_ms);
                }
                else
                {
                    PositionStallMonitor_Disarm(&upper_j4310_stall_monitor);
                }
            }
            else
            {
                J4310PositionControl_CancelTrajectory(
                    &upper_j4310_position_control);
                PositionStallMonitor_Disarm(&upper_j4310_stall_monitor);
            }
        }
        if (target.gate.enabled && target.gate.position_mode)
        {
            PositionStallMonitor_Arm(&upper_gate_stall_monitor,
                                     target.gate.m2006_pos_rad,
                                     tick_ms);
        }
        else
        {
            PositionStallMonitor_Disarm(&upper_gate_stall_monitor);
        }
        UpperEntry_ConfigureGripperStallProtection(&target.gripper, true);
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
        upper_j4310_startup_enable_pending = false;
        J4310AutoReturn_Cancel(&upper_j4310_auto_return);
        PositionStallMonitor_Disarm(&upper_j4310_stall_monitor);
        (void)UpperEntry_ResetJ4310PositionControl();
        UpperRobot_Stop(&upper_robot);
        success = UpperMotorPort_SaveJ4310Zero(can_bus, node_id);
    }
    else if ((action == UPPER_PC_ACTION_J4310_ENABLE) &&
             (can_bus == CAN_BUS_ARM_J4310) &&
             (node_id == NODE_ARM_J4310))
    {
        success = UpperMotorPort_EnableJ4310(can_bus, node_id);
    }
    else if ((action == UPPER_PC_ACTION_J4310_AUTO_RETURN) &&
             (can_bus == CAN_BUS_ARM_J4310) &&
             (node_id == NODE_ARM_J4310) && (value <= 1U))
    {
        upper_j4310_auto_return_enabled = value != 0U;
        feedback_fresh = UpperMotorPort_GetJ4310Feedback(
                             CAN_BUS_ARM_J4310,
                             NODE_ARM_J4310,
                             &feedback);
        J4310AutoReturn_Configure(&upper_j4310_auto_return,
                                  upper_j4310_auto_return_enabled,
                                  feedback_fresh);
        J4310PositionControl_Hold(
            &upper_j4310_position_control,
            feedback_fresh ? feedback.position_rad :
                             upper_robot.target.arm.grip_pos_rad);
        PositionStallMonitor_Disarm(&upper_j4310_stall_monitor);
        (void)MotorManager_ClearOverride(&upper_robot.motor_manager,
                                         UPPER_MOTOR_ARM_J4310);
        success = true;
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
/* ==================== 任务入口 ==================== */


/* 功能：初始化上层入口、机器人、链路和通信回调；用途：完成用户应用启动；返回 true 表示所有子模块初始化成功。 */
bool UpperEntry_Init(void)
{
    bool robot_ready;

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
    UpperPcLink_SetAuxControlHandler(&upper_pc_link,
                                     UpperEntry_OnPcAuxControl);
    UpperRemoteLink_Init(&upper_remote_link);
    J4310AutoReturn_Init(&upper_j4310_auto_return, false);
    if (!UpperEntry_ResetJ4310PositionControl())
    {
        return false;
    }
    if (!UpperEntry_InitStallRecovery())
    {
        return false;
    }
    upper_j4310_auto_return_enabled = false;
    upper_j4310_startup_enable_pending = true;
    upper_j4310_startup_enable_attempted = false;
    upper_j4310_startup_enable_last_tick_ms = 0U;
    upper_remote_gate_disable_pending = false;
    upper_remote_previous_primary_key_bits = 0U;
    upper_remote_previous_key_bits = 0U;
    upper_remote_previous_switch_bits = 0U;
    upper_remote_have_switch_state = false;
    upper_remote_mode = UPPER_REMOTE_MODE_STORE3_AUTO;
    UpperEntry_ResetRemoteModeState(true);
    /* PE4 初始为关闭状态；在自动流程提出请求前不发送。 */
    upper_aux_uart5_pending_count = 0U;
    upper_aux_output_bits = UPPER_AUX_OUTPUT_PE4;
    upper_aux_update_mask = 0U;
    upper_aux_uart5_sequence = 0U;
    upper_aux_uart5_sent_count = 0U;
    upper_aux_uart5_fail_count = 0U;
    CommRuntime_SetHandlers(UpperEntry_OnUart, UpperEntry_OnCan, NULL);
    robot_ready = UpperRobot_Init(&upper_robot,
                                  upper_motor_cfg,
                                  UPPER_MOTOR_COUNT,
                                  UpperMotorPort_Send,
                                  NULL);
    if (robot_ready)
    {
        UpperEntry_ConfigureGripperStallProtection(
            &upper_robot.target.gripper, true);
    }
    return robot_ready;
}

/* 功能：执行上层应用的 1 ms 主控制周期；用途：处理命令、控制电机、检查故障和通信事件；无返回值表示完成一次调度。 */
void UpperEntry_Control1ms(uint32_t tick_ms)
{
    UpperMotorPort_BeginCycle(tick_ms);
    UpperEntry_ProcessCmd(tick_ms);
    UpperEntry_ProcessRemote(tick_ms);
    UpperEntry_ProcessAuxUart5();
    UpperEntry_ProcessMotorAction(tick_ms);
    if (upper_estop_pending)
    {
        upper_estop_pending = false;
        UpperEntry_ResetRemoteAutomaticSequences();
        UpperEntry_ResetRemotePc0Sequence();
        upper_remote_pd11_pending = false;
        upper_remote_pd8_first_pending = false;
        upper_j4310_startup_enable_pending = false;
        J4310AutoReturn_Cancel(&upper_j4310_auto_return);
        J4310PositionControl_CancelTrajectory(
            &upper_j4310_position_control);
        PositionStallMonitor_Disarm(&upper_j4310_stall_monitor);
        PositionStallMonitor_Disarm(&upper_gate_stall_monitor);
        PositionStallMonitor_Disarm(&upper_gripper_stall_monitor);
        upper_gate_stall_rearm_pending = false;
        upper_gripper_stall_rearm_pending = false;
        upper_gripper_stall_protection_enabled = true;
        upper_remote_gate_disable_pending = false;
        UpperRobot_EStop(&upper_robot);
    }

    UpperEntry_ServiceJ4310StartupEnable(tick_ms);
    UpperEntry_ServiceStallRecovery(tick_ms);
    UpperEntry_ServiceRemoteGateDisable();
    UpperEntry_ServiceJ4310Control(tick_ms);
    UpperRobot_Control1ms(&upper_robot, tick_ms);
    if (!UpperMotorPort_Flush())
    {
        upper_robot.motor_manager.send_fail_count++;
    }
    UpperEntry_CheckMotorHealth(tick_ms);
    UpperEntry_ProcessPcEvents(tick_ms);
    UpperEntry_SendState(tick_ms);
    UpperEntry_SendDjiTelemetry(tick_ms);
}

/* 功能：使用 HAL 毫秒时基调用上层控制周期；用途：提供给系统任务的固定入口；无返回值表示完成本次 1 ms 调用。 */
void App_Control1ms(void)
{
    UpperEntry_Control1ms(CommRuntime_GetTickMs());
}
/* ==================== 遥控接收与总处理 ==================== */



/* 模式变化时取消流程和延时；首次自动联动历史按需保留。 */
static void UpperEntry_ResetRemoteModeState(bool clear_history)
{
    upper_remote_flip_mode = false;
    upper_remote_auto_pe4_state = UPPER_REMOTE_AUTO_PE4_IDLE;
    upper_remote_flip_action = UPPER_REMOTE_FLIP_ACTION_NONE;
    upper_remote_flip_first_stage = false;
    upper_remote_flip_due_tick_ms = 0U;
    if (clear_history)
    {
        UpperEntry_ResetRemoteAutomaticSequences();
    }
    else
    {
        UpperEntry_ResetRemoteAutomaticProgress();
    }
    upper_remote_pd13_second = false;
    upper_remote_pd12_second = false;
    upper_remote_pd11_second = false;
    upper_remote_pd11_pending = false;
    upper_remote_pd11_due_tick_ms = 0U;
    upper_remote_pd11_pending_j4310_deg =
        UPPER_REMOTE_PD11_FIRST_J4310_DEG;
    upper_remote_pd8_second = false;
    upper_remote_pd8_first_pending = false;
    upper_remote_pd8_first_due_tick_ms = 0U;
    upper_remote_pd9_zero_pending = false;
    upper_remote_pd9_second = false;
    upper_remote_pd10_second = false;
    UpperEntry_ResetRemotePc0Sequence();
}

/* 从 UART5 固定帧遥控电平快照中处理按键上升沿和开关状态。 */
/* 功能：解析遥控按键和摇杆并更新机器人目标；用途：实现遥控动作映射、边沿触发和超时处理；无返回值表示本周期遥控输入已处理。 */
static void UpperEntry_ProcessRemote(uint32_t tick_ms)
{
    upper_remote_control_t control;
    upper_remote_mode_t remote_mode;
    upper_remote_auto_pd12_state_t pd12_state_before_press;
    bool remote_online;
    bool suppress_automatic_pe4_edge;
    uint8_t primary_key_bits;
    uint8_t primary_rising_bits;
    uint8_t key_bits;
    uint8_t rising_bits;
    uint8_t switch_bits;
    uint8_t changed_switch_bits;
    bool pe4_closed;
    bool automatic_mode;
    bool pc0_reset_key_pressed;
    bool pc0_pe4_edge_consumed;

    remote_online = UpperEntry_GetSecondaryRemoteControl(&control, tick_ms) &&
                    control.online;
    if (!remote_online)
    {
        upper_remote_previous_primary_key_bits = 0U;
        upper_remote_previous_key_bits = 0U;
        upper_remote_previous_switch_bits = 0U;
        upper_remote_have_switch_state = false;
        upper_remote_mode = UPPER_REMOTE_MODE_STORE3_AUTO;
        UpperEntry_ResetRemoteModeState(true);
        return;
    }

    primary_key_bits = control.primary_key_bits &
                       UPPER_REMOTE_PRIMARY_KEY_MASK;
    primary_rising_bits = primary_key_bits &
                          (uint8_t)~upper_remote_previous_primary_key_bits;
    upper_remote_previous_primary_key_bits = primary_key_bits;
    key_bits = control.key_bits & UPPER_REMOTE_KEY_MASK;
    switch_bits = control.switch_bits & UPPER_REMOTE_SWITCH_MASK;
    rising_bits = key_bits & (uint8_t)~upper_remote_previous_key_bits;
    upper_remote_previous_key_bits = key_bits;
    if (!upper_remote_have_switch_state)
    {
        upper_remote_previous_switch_bits = switch_bits;
        upper_remote_have_switch_state = true;
        changed_switch_bits = 0U;
    }
    else
    {
        changed_switch_bits = switch_bits ^ upper_remote_previous_switch_bits;
        upper_remote_previous_switch_bits = switch_bits;
    }
    pe4_closed = (switch_bits & UPPER_REMOTE_SWITCH_PE4) != 0U;
    pc0_pe4_edge_consumed = false;
    remote_mode = UPPER_REMOTE_MODE_FROM_SWITCHES(control.primary_switch);
    if (remote_mode != upper_remote_mode)
    {
        UpperEntry_ResetRemoteModeState(false);
        upper_remote_mode = remote_mode;
    }

    automatic_mode =
        (remote_mode == UPPER_REMOTE_MODE_STORE3_AUTO) ||
        (remote_mode == UPPER_REMOTE_MODE_STORE2_AUTO);
    pc0_reset_key_pressed =
        automatic_mode &&
        ((rising_bits & (UPPER_REMOTE_KEY_PD13 |
                         UPPER_REMOTE_KEY_PD12 |
                         UPPER_REMOTE_KEY_PD11)) != 0U);
    if (pc0_reset_key_pressed)
    {
        /* 右侧按键保留原动作，并让该模式的下一次 PC0 从第一分支开始。 */
        UpperEntry_ResetRemotePc0ToFirstBranch();
    }

    if (automatic_mode &&
        !pc0_reset_key_pressed &&
        ((primary_rising_bits & UPPER_REMOTE_PRIMARY_KEY_PC0) != 0U))
    {
        UpperEntry_HandleRemotePc0Press(tick_ms);
    }
    /* PC1 是四种模式共用动作，不受 PC0 自动序列状态限制。 */
    if ((primary_rising_bits & UPPER_REMOTE_PRIMARY_KEY_PC1) != 0U)
    {
        UpperEntry_HandleRemotePc1();
    }

    if ((changed_switch_bits & UPPER_REMOTE_SWITCH_PE4) != 0U)
    {
        pc0_pe4_edge_consumed = UpperEntry_HandleRemotePc0Pe4(pe4_closed,
                                                               tick_ms);
    }

    if ((upper_remote_pc0_state != UPPER_REMOTE_PC0_IDLE) ||
        pc0_pe4_edge_consumed)
    {
        UpperEntry_ServiceRemotePc0Sequence(tick_ms);
        if (automatic_mode)
        {
            UpperEntry_HandleRemotePd9Pd10(rising_bits);
        }
        return;
    }

    if ((remote_mode == UPPER_REMOTE_MODE_STORE3_MANUAL) ||
        (remote_mode == UPPER_REMOTE_MODE_STORE2_MANUAL))
    {
        UpperEntry_PrepareRemoteActions(rising_bits);

        if ((rising_bits & UPPER_REMOTE_KEY_PD13) != 0U)
        {
            UpperEntry_HandleRemotePd13(tick_ms);
        }
        if ((rising_bits & UPPER_REMOTE_KEY_PD12) != 0U)
        {
            UpperEntry_HandleRemotePd12(tick_ms);
        }
        if ((rising_bits & UPPER_REMOTE_KEY_PD11) != 0U)
        {
            if (remote_mode == UPPER_REMOTE_MODE_STORE3_MANUAL)
            {
                UpperEntry_HandleRemotePd11(tick_ms);
            }
            else
            {
                UpperEntry_StartRemotePd11First(tick_ms);
                upper_remote_pd11_second = false;
            }
        }
        if ((rising_bits & UPPER_REMOTE_KEY_PD8) != 0U)
        {
            UpperEntry_HandleRemotePd8(tick_ms);
        }
        if ((rising_bits & UPPER_REMOTE_KEY_PD9) != 0U &&
            (upper_remote_pc0_state == UPPER_REMOTE_PC0_IDLE))
        {
            UpperEntry_HandleRemotePd9();
        }
        if ((rising_bits & UPPER_REMOTE_KEY_PD10) != 0U)
        {
            UpperEntry_HandleRemotePd10();
        }

    }
    else if ((remote_mode == UPPER_REMOTE_MODE_STORE3_AUTO) ||
             (remote_mode == UPPER_REMOTE_MODE_STORE2_AUTO))
    {
        pd12_state_before_press = upper_remote_auto_pd12_state;

        /* PD8 进入/退出自动翻转模式，不再依赖左侧按键。 */
        if ((rising_bits & UPPER_REMOTE_KEY_PD8) != 0U)
        {
            UpperEntry_PrepareRemoteActions(UPPER_REMOTE_KEY_PD8);
            upper_remote_flip_mode = !upper_remote_flip_mode;
            upper_remote_auto_pe4_state = UPPER_REMOTE_AUTO_PE4_IDLE;
            upper_remote_flip_action = UPPER_REMOTE_FLIP_ACTION_NONE;
            upper_remote_flip_first_stage = false;
            upper_remote_flip_due_tick_ms = 0U;
            upper_remote_pd8_first_pending = false;
            upper_remote_pd13_second = false;
            upper_remote_pd12_second = false;
            UpperEntry_ResetRemoteAutomaticSequences();
        }

        if ((rising_bits & UPPER_REMOTE_KEY_PD13) != 0U)
        {
            if (upper_remote_flip_mode)
            {
                UpperEntry_HandleRemoteFlipPress(
                    UPPER_REMOTE_FLIP_ACTION_PD13, tick_ms);
            }
            else
            {
                /* 自动模式下每次按 PD13 都先明确打开 PE4，再选择执行分支。 */
                UpperEntry_OpenRemotePe4Output();
                upper_remote_auto_pe4_state = UPPER_REMOTE_AUTO_PE4_IDLE;
                UpperEntry_PrepareRemoteActions(UPPER_REMOTE_KEY_PD13);
                /* 清除 PD12 当前流程，PD12/PC0 的已按历史均保留。 */
                UpperEntry_ResetRemoteAutoPd12Sequence();
                upper_remote_mode_history[upper_remote_mode].
                    auto_pd12_branch_two_armed = false;
                UpperEntry_HandleRemoteAutoPd13Press(tick_ms);
            }
        }
        if ((rising_bits & UPPER_REMOTE_KEY_PD12) != 0U)
        {
            if (upper_remote_flip_mode)
            {
                UpperEntry_HandleRemoteFlipPress(
                    UPPER_REMOTE_FLIP_ACTION_PD12, tick_ms);
            }
            else
            {
                /* 自动模式下每次按 PD12 都先明确打开 PE4，再选择执行分支。 */
                UpperEntry_OpenRemotePe4Output();
                upper_remote_auto_pe4_state = UPPER_REMOTE_AUTO_PE4_IDLE;
                UpperEntry_PrepareRemoteActions(UPPER_REMOTE_KEY_PD12);
                /* 仅清除 PD13 流程，保留其“曾按过”历史。 */
                UpperEntry_ResetRemoteAutoPd13Sequence();
                upper_remote_mode_history[upper_remote_mode].
                    auto_pd13_branch_two_armed = false;
                UpperEntry_HandleRemoteAutoPd12Press(tick_ms);
            }
        }
        if ((rising_bits & UPPER_REMOTE_KEY_PD11) != 0U)
        {
            upper_remote_auto_pe4_state = UPPER_REMOTE_AUTO_PE4_IDLE;
            upper_remote_flip_action = UPPER_REMOTE_FLIP_ACTION_NONE;
            upper_remote_flip_first_stage = false;
            upper_remote_flip_due_tick_ms = 0U;
            UpperEntry_HandleRemoteAutoPd11Press(tick_ms);
        }

        /*
         * PD13/PD12 按键用于选择分支时，不让同帧 PE4 边沿立即收尾
         * 刚进入的分支二。PD12 分支一最终 PE4 与同键同帧时仍允许
         * 完成；其同键请求会被锁存并在分支一结束后进入分支二。
         */
        suppress_automatic_pe4_edge =
            (rising_bits & (UPPER_REMOTE_KEY_PD13 |
                            UPPER_REMOTE_KEY_PD12)) != 0U;
        if (((rising_bits & UPPER_REMOTE_KEY_PD12) != 0U) &&
            ((rising_bits & UPPER_REMOTE_KEY_PD13) == 0U) &&
            (pd12_state_before_press ==
             UPPER_REMOTE_AUTO_PD12_WAIT_FINAL_PE4))
        {
            suppress_automatic_pe4_edge = false;
        }

        if (((changed_switch_bits & UPPER_REMOTE_SWITCH_PE4) != 0U) &&
            !pc0_pe4_edge_consumed)
        {
            if (upper_remote_flip_mode && pe4_closed &&
                (upper_remote_auto_pe4_state ==
                 UPPER_REMOTE_AUTO_PE4_WAIT_RESET))
            {
                UpperEntry_PrepareRemoteActions(UPPER_REMOTE_KEY_PD8);
                if (upper_remote_flip_first_stage)
                {
                    upper_remote_flip_due_tick_ms =
                        tick_ms + UPPER_REMOTE_FLIP_FIRST_CLOSE_DELAY_MS;
                    upper_remote_auto_pe4_state =
                        UPPER_REMOTE_AUTO_PE4_WAIT_FIRST_CLOSE_DELAY;
                }
                else
                {
                    UpperEntry_StartRemoteAutoFlipReset(tick_ms);
                    upper_remote_auto_pe4_state =
                        (upper_remote_flip_action ==
                         UPPER_REMOTE_FLIP_ACTION_PD13) ?
                            UPPER_REMOTE_AUTO_PE4_WAIT_PD13 :
                            UPPER_REMOTE_AUTO_PE4_WAIT_PD12;
                }
            }
            else if (!upper_remote_flip_mode &&
                     !suppress_automatic_pe4_edge)
            {
                UpperEntry_HandleRemoteAutomaticPe4(pe4_closed, tick_ms);
            }
        }

        UpperEntry_ServiceRemoteFlipSequence(tick_ms);

        /* 自动模式保留 PD9、PD10 的原手动动作。 */
        UpperEntry_HandleRemotePd9Pd10(rising_bits);

        if (upper_remote_pc0_state == UPPER_REMOTE_PC0_IDLE)
        {
            UpperEntry_ServiceRemoteAutomaticSequences(tick_ms);
        }
    }

    if (upper_remote_pd11_pending &&
        ((int32_t)(tick_ms - upper_remote_pd11_due_tick_ms) >= 0))
    {
        UpperEntry_ApplyRemoteJ4310(
            upper_remote_pd11_pending_j4310_deg, tick_ms);
        upper_remote_pd11_pending = false;
    }
    if (upper_remote_pd8_first_pending &&
        ((int32_t)(tick_ms - upper_remote_pd8_first_due_tick_ms) >= 0))
    {
        UpperEntry_ApplyRemoteJ4310(
            UPPER_REMOTE_PD8_FIRST_J4310_DEG, tick_ms);
        upper_remote_pd8_first_pending = false;
    }

    UpperEntry_ServiceRemotePc0Sequence(tick_ms);
}

/* 功能：取得遥控第二组控制快照；用途：向上层机构逻辑提供按键、开关和在线状态。 */
bool UpperEntry_GetSecondaryRemoteControl(upper_remote_control_t *control,
                                          uint32_t tick_ms)
{
    return UpperRemoteLink_GetControl(&upper_remote_link, tick_ms, control);
}

/* 功能：读取副遥控链路诊断信息；用途：向监控或调试模块提供收帧统计；无返回值表示诊断数据已复制。 */
void UpperEntry_GetSecondaryRemoteDiagnostics(
    upper_remote_diagnostics_t *diagnostics)
{
    UpperRemoteLink_GetDiagnostics(&upper_remote_link, diagnostics);
}
/* ==================== PE4、PE3、PE1、PE0、PD6、PD5 拨动开关 ==================== */


/* PE4 辅助状态使用 0=打开、1=关闭，不改变其余辅助输出位。 */
static void UpperEntry_OpenRemotePe4Output(void)
{
    upper_aux_output_bits &= (uint8_t)~UPPER_AUX_OUTPUT_PE4;
    upper_aux_update_mask |= UPPER_AUX_OUTPUT_PE4;
    upper_aux_uart5_pending_count = UPPER_AUX_UART_REPEAT_COUNT;
}

/* 功能：关闭遥控流程控制的 PE4 辅助输出；用途：置位输出状态并安排 UART5 重复转发。 */
static void UpperEntry_CloseRemotePe4Output(void)
{
    upper_aux_output_bits |= UPPER_AUX_OUTPUT_PE4;
    upper_aux_update_mask |= UPPER_AUX_OUTPUT_PE4;
    upper_aux_uart5_pending_count = UPPER_AUX_UART_REPEAT_COUNT;
}

/* PD13、PD12 的等待状态只接受手动关闭 PE4 的边沿。 */
static void UpperEntry_HandleRemoteAutomaticPe4(bool pe4_closed,
                                                uint32_t tick_ms)
{
    if (pe4_closed &&
        (upper_remote_auto_pd13_state ==
         UPPER_REMOTE_AUTO_PD13_WAIT_FIRST_PE4))
    {
        UpperEntry_PrepareRemoteActions(UPPER_REMOTE_KEY_PD11);
        UpperEntry_ApplyRemoteM3508(UPPER_REMOTE_PD11_FIRST_M3508_DEG);
        upper_remote_auto_pd13_due_tick_ms =
            tick_ms + UPPER_REMOTE_FINAL_J4310_DELAY_MS;
        upper_remote_auto_pd13_state =
            UPPER_REMOTE_AUTO_PD13_WAIT_BRANCH_ONE_FINAL_J4310;
    }
    else if (pe4_closed &&
             (upper_remote_auto_pd13_state ==
              UPPER_REMOTE_AUTO_PD13_WAIT_DIRECT_SECOND_PE4))
    {
        UpperEntry_PrepareRemoteActions(UPPER_REMOTE_KEY_PD11);
        UpperEntry_ApplyRemoteM3508(UPPER_REMOTE_PD11_FIRST_M3508_DEG);
        upper_remote_auto_pd13_due_tick_ms =
            tick_ms + UPPER_REMOTE_FINAL_J4310_DELAY_MS;
        upper_remote_auto_pd13_state =
            UPPER_REMOTE_AUTO_PD13_WAIT_DIRECT_SECOND_J4310;
    }

    if (pe4_closed &&
        (upper_remote_auto_pd12_state ==
         UPPER_REMOTE_AUTO_PD12_WAIT_FIRST_PE4_OR_SECOND_PRESS))
    {
        if (UPPER_REMOTE_AUTO_HAS_240_STAGE(upper_remote_mode))
        {
            UpperEntry_PrepareRemoteActions(UPPER_REMOTE_KEY_PD11);
            UpperEntry_ApplyRemoteM3508(UPPER_REMOTE_PD11_SECOND_M3508_DEG);
            upper_remote_auto_pd12_j4310_due_tick_ms =
                tick_ms + UPPER_REMOTE_FINAL_J4310_DELAY_MS;
            upper_remote_auto_pd12_j4310_pending = true;
            upper_remote_auto_pd12_state =
                UPPER_REMOTE_AUTO_PD12_WAIT_PD11_J4310;
        }
        else
        {
            /* 存二自动首次 PE4 关闭后，500 ms 收尾到 180 度。 */
            UpperEntry_PrepareRemoteActions(UPPER_REMOTE_KEY_PD11);
            UpperEntry_ApplyRemoteM3508(UPPER_REMOTE_PD11_FIRST_M3508_DEG);
            upper_remote_auto_pd12_j4310_due_tick_ms =
                tick_ms + UPPER_REMOTE_FINAL_J4310_DELAY_MS;
            upper_remote_auto_pd12_j4310_pending = true;
            upper_remote_auto_pd12_state =
                UPPER_REMOTE_AUTO_PD12_WAIT_BRANCH_ONE_FINAL_J4310;
        }
    }
    else if (pe4_closed &&
             (upper_remote_auto_pd12_state ==
              UPPER_REMOTE_AUTO_PD12_WAIT_FINAL_PE4))
    {
        UpperEntry_PrepareRemoteActions(UPPER_REMOTE_KEY_PD11);
        UpperEntry_ApplyRemoteM3508(UPPER_REMOTE_PD11_FIRST_M3508_DEG);
        upper_remote_auto_pd12_j4310_due_tick_ms =
            tick_ms + UPPER_REMOTE_FINAL_J4310_DELAY_MS;
        upper_remote_auto_pd12_j4310_pending = true;
        upper_remote_auto_pd12_state =
            UPPER_REMOTE_AUTO_PD12_WAIT_BRANCH_ONE_FINAL_J4310;
    }
    else if (pe4_closed &&
             (upper_remote_auto_pd12_state ==
              UPPER_REMOTE_AUTO_PD12_WAIT_DIRECT_SECOND_PE4))
    {
        UpperEntry_PrepareRemoteActions(UPPER_REMOTE_KEY_PD11);
        UpperEntry_ApplyRemoteM3508(UPPER_REMOTE_PD11_FIRST_M3508_DEG);
        upper_remote_auto_pd12_j4310_due_tick_ms =
            tick_ms + UPPER_REMOTE_FINAL_J4310_DELAY_MS;
        upper_remote_auto_pd12_j4310_pending = true;
        upper_remote_auto_pd12_state =
            UPPER_REMOTE_AUTO_PD12_WAIT_BRANCH_TWO_FINAL_J4310;
    }
}
/* ==================== 遥控自动控制 ==================== */


static bool UpperEntry_IsFirstRemoteAutomaticStoragePress(
    const upper_remote_mode_history_t *history)
{
    return (history != NULL) && UPPER_REMOTE_AUTO_START_IS_AVAILABLE(
               history->auto_pd13_has_pressed,
               history->auto_pd12_has_pressed,
               history->auto_pc0_has_pressed);
}

/* 本轮 PD13、PD12、PC0 的首次按下共享 PE4 起始动作；存二自动同时复用 PD10 首段。 */
static void UpperEntry_ApplyRemoteAutomaticStartOutputs(
    bool first_storage_press)
{
    if (!first_storage_press)
    {
        return;
    }

    UpperEntry_OpenRemotePe4Output();
    if (upper_remote_mode == UPPER_REMOTE_MODE_STORE2_AUTO)
    {
        upper_remote_pd10_second = false;
        UpperEntry_HandleRemotePd10();
        return;
    }

    UpperEntry_ApplyRemoteGripper(
        UPPER_REMOTE_AUTO_START_GRIPPER_DEG,
        UPPER_REMOTE_AUTO_START_GRIPPER_STALL_PROTECTION != 0U);
    upper_remote_pd10_second = false;
}

/* 启动翻转子流程：下发首段机械臂目标并自动打开 PE4。 */
static void UpperEntry_StartRemoteFlipAction(
    upper_remote_flip_action_t action,
    uint32_t tick_ms)
{
    upper_remote_flip_action = action;
    upper_remote_flip_first_stage = true;
    upper_remote_flip_due_tick_ms = 0U;
    UpperEntry_OpenRemotePe4Output();
    if (action == UPPER_REMOTE_FLIP_ACTION_PD13)
    {
        upper_remote_pd13_second = false;
        upper_remote_pd12_second = false;
        UpperEntry_PrepareRemoteActions(UPPER_REMOTE_KEY_PD13);
        UpperEntry_ApplyRemotePd13First(tick_ms);
    }
    else
    {
        upper_remote_pd12_second = false;
        upper_remote_pd13_second = false;
        UpperEntry_PrepareRemoteActions(UPPER_REMOTE_KEY_PD12);
        UpperEntry_ApplyRemotePd12First(tick_ms);
    }
    upper_remote_pd10_second = false;
    UpperEntry_HandleRemotePd10();
    upper_remote_auto_pe4_state = UPPER_REMOTE_AUTO_PE4_WAIT_RESET;
}

/* 翻转子流程收尾：确认键到达后执行 J4310=40 度，首段延时后再打开 PE4。 */
static void UpperEntry_FinishRemoteFlipAction(uint32_t tick_ms)
{
    UpperEntry_PrepareRemoteActions(UPPER_REMOTE_KEY_PD8);
    if (upper_remote_flip_first_stage)
    {
        UpperEntry_ApplyRemoteM3508(
            UPPER_REMOTE_FLIP_FIRST_FINAL_M3508_DEG);
    }
    UpperEntry_ApplyRemoteJ4310(UPPER_REMOTE_PD8_FIRST_J4310_DEG,
                                tick_ms);
    if (upper_remote_flip_first_stage)
    {
        upper_remote_flip_due_tick_ms =
            tick_ms + UPPER_REMOTE_FLIP_FIRST_FINAL_OPEN_DELAY_MS;
        upper_remote_auto_pe4_state =
            UPPER_REMOTE_AUTO_PE4_WAIT_FINAL_OPEN_DELAY;
        return;
    }

    UpperEntry_OpenRemotePe4Output();
    upper_remote_auto_pe4_state = UPPER_REMOTE_AUTO_PE4_IDLE;
    upper_remote_flip_action = UPPER_REMOTE_FLIP_ACTION_NONE;
    upper_remote_flip_first_stage = false;
    upper_remote_flip_due_tick_ms = 0U;
    upper_remote_pd13_second = false;
    upper_remote_pd12_second = false;
}

/* 翻转模式按键处理：关闭 PE4 前任一键推进当前流程，关闭后只接受同键收尾。 */
static void UpperEntry_HandleRemoteFlipPress(
    upper_remote_flip_action_t action,
    uint32_t tick_ms)
{
    if (action == UPPER_REMOTE_FLIP_ACTION_PD13)
    {
        /* PD13 会让下一次 PD12 从第一段开始。 */
        upper_remote_pd12_second = false;
    }
    else if (action == UPPER_REMOTE_FLIP_ACTION_PD12)
    {
        /* PD12 会让下一次 PD13 从第一段开始。 */
        upper_remote_pd13_second = false;
    }

    if (upper_remote_auto_pe4_state == UPPER_REMOTE_AUTO_PE4_WAIT_RESET)
    {
        UpperEntry_PrepareRemoteActions(action ==
                                        UPPER_REMOTE_FLIP_ACTION_PD13 ?
                                            UPPER_REMOTE_KEY_PD13 :
                                            UPPER_REMOTE_KEY_PD12);
        if (upper_remote_flip_action == UPPER_REMOTE_FLIP_ACTION_PD13)
        {
            UpperEntry_ApplyRemoteArm(
                UPPER_REMOTE_FLIP_PD13_NEXT_M3508_DEG,
                UPPER_REMOTE_FLIP_PD13_NEXT_J4310_DEG,
                tick_ms);
        }
        else if (upper_remote_flip_action ==
                 UPPER_REMOTE_FLIP_ACTION_PD12)
        {
            UpperEntry_ApplyRemoteArm(
                UPPER_REMOTE_FLIP_PD12_NEXT_M3508_DEG,
                UPPER_REMOTE_FLIP_PD12_NEXT_J4310_DEG,
                tick_ms);
        }
        upper_remote_flip_first_stage = false;
        return;
    }

    if ((upper_remote_auto_pe4_state == UPPER_REMOTE_AUTO_PE4_WAIT_PD13) &&
        (action == UPPER_REMOTE_FLIP_ACTION_PD13))
    {
        UpperEntry_FinishRemoteFlipAction(tick_ms);
    }
    else if ((upper_remote_auto_pe4_state ==
              UPPER_REMOTE_AUTO_PE4_WAIT_PD12) &&
             (action == UPPER_REMOTE_FLIP_ACTION_PD12))
    {
        UpperEntry_FinishRemoteFlipAction(tick_ms);
    }
    else if (upper_remote_auto_pe4_state == UPPER_REMOTE_AUTO_PE4_IDLE)
    {
        UpperEntry_StartRemoteFlipAction(action, tick_ms);
    }
}

/* 翻转模式的 PE4 关闭阶段：闸门和 M3508 归零，等待对应按键确认。 */
static void UpperEntry_StartRemoteAutoFlipReset(uint32_t tick_ms)
{
    UpperEntry_ApplyRemoteGate(UPPER_REMOTE_PD9_ZERO_GATE_DEG);
    UpperEntry_ApplyRemoteM3508(UPPER_REMOTE_PD8_FIRST_M3508_DEG);
    if (upper_remote_flip_first_stage)
    {
        UpperEntry_ApplyRemoteJ4310(
            UPPER_REMOTE_FLIP_FIRST_CLOSE_J4310_DEG,
            tick_ms);
    }
    upper_remote_pd9_zero_pending = false;
    upper_remote_pd9_second = false;
    upper_remote_pd8_first_pending = false;
    upper_remote_pd8_first_due_tick_ms = 0U;
}

/* 翻转首段的两个定时阶段：关闭后延时归零，以及 J4310=40 后延时打开 PE4。 */
static void UpperEntry_ServiceRemoteFlipSequence(uint32_t tick_ms)
{
    if ((upper_remote_auto_pe4_state ==
         UPPER_REMOTE_AUTO_PE4_WAIT_FIRST_CLOSE_DELAY) &&
        ((int32_t)(tick_ms - upper_remote_flip_due_tick_ms) >= 0))
    {
        UpperEntry_StartRemoteAutoFlipReset(tick_ms);
        upper_remote_flip_due_tick_ms = 0U;
        upper_remote_auto_pe4_state =
            (upper_remote_flip_action == UPPER_REMOTE_FLIP_ACTION_PD13) ?
                UPPER_REMOTE_AUTO_PE4_WAIT_PD13 :
                UPPER_REMOTE_AUTO_PE4_WAIT_PD12;
    }
    else if ((upper_remote_auto_pe4_state ==
              UPPER_REMOTE_AUTO_PE4_WAIT_FINAL_OPEN_DELAY) &&
             ((int32_t)(tick_ms - upper_remote_flip_due_tick_ms) >= 0))
    {
        UpperEntry_OpenRemotePe4Output();
        upper_remote_auto_pe4_state = UPPER_REMOTE_AUTO_PE4_IDLE;
        upper_remote_flip_action = UPPER_REMOTE_FLIP_ACTION_NONE;
        upper_remote_flip_first_stage = false;
        upper_remote_flip_due_tick_ms = 0U;
        upper_remote_pd13_second = false;
        upper_remote_pd12_second = false;
    }
}

/* 功能：复位自动翻转模式下的 PD13 时序；用途：取消当前阶段并清除到期时间。 */
static void UpperEntry_ResetRemoteAutoPd13Sequence(void)
{
    upper_remote_auto_pd13_state = UPPER_REMOTE_AUTO_PD13_IDLE;
    upper_remote_auto_pd13_due_tick_ms = 0U;
    upper_remote_auto_pd13_direct_second_pending = false;
}

/* 功能：复位自动翻转模式下的 PD12 时序；用途：取消状态机及待执行的 J4310 延时动作。 */
static void UpperEntry_ResetRemoteAutoPd12Sequence(void)
{
    upper_remote_auto_pd12_state = UPPER_REMOTE_AUTO_PD12_IDLE;
    upper_remote_auto_pd12_j4310_pending = false;
    upper_remote_auto_pd12_j4310_due_tick_ms = 0U;
    upper_remote_auto_pd12_direct_second_pending = false;
}

/* 清除自动 PD11 的单双按流程和到期时间。 */
static void UpperEntry_ResetRemoteAutoPd11Sequence(void)
{
    upper_remote_auto_pd11_state = UPPER_REMOTE_AUTO_PD11_IDLE;
    upper_remote_auto_pd11_due_tick_ms = 0U;
}

/* 清除非翻转模式下 PD13、PD12、PD11 的当前流程，不修改模式历史。 */
static void UpperEntry_ResetRemoteAutomaticProgress(void)
{
    UpperEntry_ResetRemoteAutoPd13Sequence();
    UpperEntry_ResetRemoteAutoPd12Sequence();
    UpperEntry_ResetRemoteAutoPd11Sequence();
}

/* 整体复位时同时清除所有模式已经触发过的首次自动联动历史。 */
static void UpperEntry_ResetRemoteAutomaticSequences(void)
{
    UpperEntry_ResetRemoteAutomaticProgress();
    (void)memset(upper_remote_mode_history,
                 0,
                 sizeof(upper_remote_mode_history));
}

/* 同键第二次按下或分支一结束后的已锁存按下，都从这里进入 PD13 分支二。 */
static void UpperEntry_StartRemoteAutoPd13DirectSecond(uint32_t tick_ms)
{
    upper_remote_mode_history[upper_remote_mode].
        auto_pd13_branch_two_armed = false;
    upper_remote_auto_pd13_direct_second_pending = false;
    upper_remote_auto_pd13_due_tick_ms = 0U;
    UpperEntry_PrepareRemoteActions(UPPER_REMOTE_KEY_PD13);
    UpperEntry_ApplyRemotePd13Second(tick_ms);
    upper_remote_auto_pd13_state =
        UPPER_REMOTE_AUTO_PD13_WAIT_DIRECT_SECOND_PE4;
}

/* 同键第二次按下或分支一结束后的已锁存按下，都从这里进入 PD12 分支二。 */
static void UpperEntry_StartRemoteAutoPd12DirectSecond(uint32_t tick_ms)
{
    upper_remote_mode_history[upper_remote_mode].
        auto_pd12_branch_two_armed = false;
    upper_remote_auto_pd12_direct_second_pending = false;
    upper_remote_auto_pd12_j4310_pending = false;
    upper_remote_auto_pd12_j4310_due_tick_ms = 0U;
    UpperEntry_PrepareRemoteActions(UPPER_REMOTE_KEY_PD12);
    UpperEntry_ApplyRemotePd12Second(tick_ms);
    upper_remote_auto_pd12_state =
        UPPER_REMOTE_AUTO_PD12_WAIT_DIRECT_SECOND_PE4;
}

/* 自动 PD13 按既定存块时序运行，内部仍使用手动按键的固定动作段。 */
static void UpperEntry_HandleRemoteAutoPd13Press(uint32_t tick_ms)
{
    upper_remote_mode_history_t *history;
    bool first_storage_press;

    history = &upper_remote_mode_history[upper_remote_mode];
    first_storage_press =
        UpperEntry_IsFirstRemoteAutomaticStoragePress(history);
    history->auto_pd13_has_pressed = true;
    if (history->auto_pd13_branch_two_armed)
    {
        UpperEntry_StartRemoteAutoPd13DirectSecond(tick_ms);
        return;
    }
    if (upper_remote_auto_pd13_state == UPPER_REMOTE_AUTO_PD13_IDLE)
    {
        UpperEntry_PrepareRemoteActions(UPPER_REMOTE_KEY_PD13);
        UpperEntry_ApplyRemotePd13First(tick_ms);
        UpperEntry_ApplyRemoteAutomaticStartOutputs(first_storage_press);
        upper_remote_auto_pd13_state =
            UPPER_REMOTE_AUTO_PD13_WAIT_FIRST_PE4;
    }
    else if ((upper_remote_auto_pd13_state ==
              UPPER_REMOTE_AUTO_PD13_WAIT_FIRST_PE4) ||
             (upper_remote_auto_pd13_state ==
              UPPER_REMOTE_AUTO_PD13_WAIT_DIRECT_SECOND_PRESS))
    {
        UpperEntry_StartRemoteAutoPd13DirectSecond(tick_ms);
    }
    else if ((upper_remote_auto_pd13_state ==
              UPPER_REMOTE_AUTO_PD13_WAIT_DIRECT_SECOND_PE4) ||
             (upper_remote_auto_pd13_state ==
              UPPER_REMOTE_AUTO_PD13_WAIT_DIRECT_SECOND_J4310))
    {
        UpperEntry_ResetRemoteAutoPd13Sequence();
        UpperEntry_PrepareRemoteActions(UPPER_REMOTE_KEY_PD13);
        UpperEntry_ApplyRemotePd13First(tick_ms);
        upper_remote_auto_pd13_state =
            UPPER_REMOTE_AUTO_PD13_WAIT_FIRST_PE4;
    }
    else if (upper_remote_auto_pd13_state ==
             UPPER_REMOTE_AUTO_PD13_WAIT_BRANCH_ONE_FINAL_J4310)
    {
        upper_remote_auto_pd13_direct_second_pending = true;
    }
}

/* 自动 PD12 根据第二次按键前是否出现 PE4 关闭边沿选择两条分支。 */
static void UpperEntry_HandleRemoteAutoPd12Press(uint32_t tick_ms)
{
    upper_remote_mode_history_t *history;
    bool first_storage_press;

    history = &upper_remote_mode_history[upper_remote_mode];
    first_storage_press =
        UpperEntry_IsFirstRemoteAutomaticStoragePress(history);
    history->auto_pd12_has_pressed = true;
    if (history->auto_pd12_branch_two_armed)
    {
        UpperEntry_StartRemoteAutoPd12DirectSecond(tick_ms);
        return;
    }
    if (upper_remote_auto_pd12_state == UPPER_REMOTE_AUTO_PD12_IDLE)
    {
        UpperEntry_PrepareRemoteActions(UPPER_REMOTE_KEY_PD12);
        UpperEntry_ApplyRemotePd12First(tick_ms);
        UpperEntry_ApplyRemoteAutomaticStartOutputs(first_storage_press);
        upper_remote_auto_pd12_state =
            UPPER_REMOTE_AUTO_PD12_WAIT_FIRST_PE4_OR_SECOND_PRESS;
    }
    else if ((upper_remote_auto_pd12_state ==
              UPPER_REMOTE_AUTO_PD12_WAIT_FIRST_PE4_OR_SECOND_PRESS) ||
             (upper_remote_auto_pd12_state ==
              UPPER_REMOTE_AUTO_PD12_WAIT_DIRECT_SECOND_PRESS))
    {
        UpperEntry_StartRemoteAutoPd12DirectSecond(tick_ms);
    }
    else if ((upper_remote_auto_pd12_state ==
              UPPER_REMOTE_AUTO_PD12_WAIT_DIRECT_SECOND_PE4) ||
             (upper_remote_auto_pd12_state ==
              UPPER_REMOTE_AUTO_PD12_WAIT_BRANCH_TWO_FINAL_J4310))
    {
        UpperEntry_ResetRemoteAutoPd12Sequence();
        UpperEntry_PrepareRemoteActions(UPPER_REMOTE_KEY_PD12);
        UpperEntry_ApplyRemotePd12First(tick_ms);
        upper_remote_auto_pd12_state =
            UPPER_REMOTE_AUTO_PD12_WAIT_FIRST_PE4_OR_SECOND_PRESS;
    }
    else if ((upper_remote_auto_pd12_state ==
              UPPER_REMOTE_AUTO_PD12_WAIT_PD11_J4310) ||
             (upper_remote_auto_pd12_state ==
              UPPER_REMOTE_AUTO_PD12_WAIT_AUTO_RESET_DELAY) ||
             (upper_remote_auto_pd12_state ==
              UPPER_REMOTE_AUTO_PD12_WAIT_FINAL_PE4) ||
             (upper_remote_auto_pd12_state ==
              UPPER_REMOTE_AUTO_PD12_WAIT_BRANCH_ONE_FINAL_J4310))
    {
        upper_remote_auto_pd12_direct_second_pending = true;
    }
}

/* 在双击窗口内检测到第二次 PD11 上升沿时，启动存二双击流程。 */
static void UpperEntry_StartRemoteStore2Pd11DoubleClick(uint32_t tick_ms)
{
    UpperEntry_ResetRemoteAutoPd13Sequence();
    UpperEntry_ResetRemoteAutoPd12Sequence();
    UpperEntry_PrepareRemoteActions(UPPER_REMOTE_KEY_PD11);
    UpperEntry_ApplyRemoteM3508(UPPER_REMOTE_STORE2_PD11_DOUBLE_M3508_DEG);
    UpperEntry_ApplyRemoteJ4310(UPPER_REMOTE_STORE2_PD11_DOUBLE_J4310_DEG,
                                tick_ms);
    upper_remote_auto_pd11_due_tick_ms =
        tick_ms + UPPER_REMOTE_STORE2_PD11_RETURN_DELAY_MS;
    upper_remote_auto_pd11_state =
        UPPER_REMOTE_AUTO_PD11_WAIT_DOUBLE_RETURN;
}

/* 启动原有的自动 PD11 单击流程。存二由双击窗口超时调用，存三按下后立即调用。 */
static void UpperEntry_StartRemoteAutoPd11SinglePress(uint32_t tick_ms)
{
    UpperEntry_PrepareRemoteActions(UPPER_REMOTE_KEY_PD11);
    UpperEntry_ApplyRemoteArm(UPPER_REMOTE_PD13_FIRST_M3508_DEG,
                              UPPER_REMOTE_PD13_FIRST_J4310_DEG,
                              tick_ms);
    UpperEntry_ApplyRemoteGate(
        (upper_remote_mode == UPPER_REMOTE_MODE_STORE2_AUTO) ?
            UPPER_REMOTE_STORE2_PD11_GATE_DEG : UPPER_REMOTE_PD11_GATE_DEG);
    upper_remote_pd9_zero_pending = false;
    /* 下一次 PD9 先去 180 度，随后恢复 60/180 度交替。 */
    upper_remote_pd9_second = false;
    upper_remote_auto_pd11_due_tick_ms =
        tick_ms + UPPER_REMOTE_PD11_OPEN_DELAY_MS;
    upper_remote_auto_pd11_state =
        UPPER_REMOTE_AUTO_PD11_WAIT_FIRST_DELAY;
}

/* 自动存二在 500 ms 窗口内区分单击/双击；自动存三保持原来的立即单击逻辑。 */
static void UpperEntry_HandleRemoteAutoPd11Press(uint32_t tick_ms)
{
    if (upper_remote_mode == UPPER_REMOTE_MODE_STORE2_AUTO)
    {
        if (upper_remote_auto_pd11_state == UPPER_REMOTE_AUTO_PD11_IDLE)
        {
            upper_remote_auto_pd11_due_tick_ms =
                tick_ms + UPPER_REMOTE_STORE2_PD11_DOUBLE_CLICK_MS;
            upper_remote_auto_pd11_state =
                UPPER_REMOTE_AUTO_PD11_WAIT_SECOND_CLICK;
        }
        else if (upper_remote_auto_pd11_state ==
                 UPPER_REMOTE_AUTO_PD11_WAIT_SECOND_CLICK)
        {
            UpperEntry_StartRemoteStore2Pd11DoubleClick(tick_ms);
        }
        return;
    }

    if (upper_remote_auto_pd11_state == UPPER_REMOTE_AUTO_PD11_IDLE)
    {
        UpperEntry_StartRemoteAutoPd11SinglePress(tick_ms);
    }
}

/* 功能：推进遥控自动动作状态机；用途：到达预定时刻后执行 PD13、PD12 和 J4310 的后续分步动作。 */
static void UpperEntry_ServiceRemoteAutomaticSequences(uint32_t tick_ms)
{
    if ((upper_remote_auto_pd13_state ==
         UPPER_REMOTE_AUTO_PD13_WAIT_BRANCH_ONE_FINAL_J4310) &&
        ((int32_t)(tick_ms - upper_remote_auto_pd13_due_tick_ms) >= 0))
    {
        UpperEntry_ApplyRemoteJ4310(
            UPPER_REMOTE_AUTO_FINAL_J4310_DEG(upper_remote_mode),
            tick_ms);
        upper_remote_mode_history[upper_remote_mode].
            auto_pd13_branch_two_armed = true;
        upper_remote_auto_pd13_state =
            UPPER_REMOTE_AUTO_PD13_WAIT_DIRECT_SECOND_PRESS;
        if (upper_remote_auto_pd13_direct_second_pending)
        {
            UpperEntry_StartRemoteAutoPd13DirectSecond(tick_ms);
        }
    }
    else if ((upper_remote_auto_pd13_state ==
              UPPER_REMOTE_AUTO_PD13_WAIT_DIRECT_SECOND_J4310) &&
             ((int32_t)(tick_ms - upper_remote_auto_pd13_due_tick_ms) >= 0))
    {
        UpperEntry_ApplyRemoteJ4310(
            UPPER_REMOTE_AUTO_FINAL_J4310_DEG(upper_remote_mode),
            tick_ms);
        upper_remote_auto_pd13_state = UPPER_REMOTE_AUTO_PD13_IDLE;
    }

    if (upper_remote_auto_pd12_j4310_pending &&
        ((int32_t)(tick_ms -
                   upper_remote_auto_pd12_j4310_due_tick_ms) >= 0))
    {
        if (upper_remote_auto_pd12_state ==
            UPPER_REMOTE_AUTO_PD12_WAIT_PD11_J4310)
        {
            UpperEntry_ApplyRemoteJ4310(
                UPPER_REMOTE_PD11_SECOND_J4310_DEG, tick_ms);
            upper_remote_auto_pd12_j4310_due_tick_ms =
                tick_ms + UPPER_REMOTE_PD12_240_HOLD_MS;
            upper_remote_auto_pd12_state =
                UPPER_REMOTE_AUTO_PD12_WAIT_AUTO_RESET_DELAY;
        }
        else if (upper_remote_auto_pd12_state ==
                 UPPER_REMOTE_AUTO_PD12_WAIT_BRANCH_ONE_FINAL_J4310)
        {
            UpperEntry_ApplyRemoteJ4310(
                UPPER_REMOTE_AUTO_FINAL_J4310_DEG(upper_remote_mode),
                tick_ms);
            upper_remote_mode_history[upper_remote_mode].
                auto_pd12_branch_two_armed = true;
            upper_remote_auto_pd12_state =
                UPPER_REMOTE_AUTO_PD12_WAIT_DIRECT_SECOND_PRESS;
            if (upper_remote_auto_pd12_direct_second_pending)
            {
                UpperEntry_StartRemoteAutoPd12DirectSecond(tick_ms);
            }
        }
        else if (upper_remote_auto_pd12_state ==
                 UPPER_REMOTE_AUTO_PD12_WAIT_BRANCH_TWO_FINAL_J4310)
        {
            UpperEntry_ApplyRemoteJ4310(
                UPPER_REMOTE_AUTO_FINAL_J4310_DEG(upper_remote_mode),
                tick_ms);
            upper_remote_auto_pd12_state = UPPER_REMOTE_AUTO_PD12_IDLE;
        }
        upper_remote_auto_pd12_j4310_pending = false;
    }
    if ((upper_remote_auto_pd12_state ==
         UPPER_REMOTE_AUTO_PD12_WAIT_AUTO_RESET_DELAY) &&
        ((int32_t)(tick_ms -
                   upper_remote_auto_pd12_j4310_due_tick_ms) >= 0))
    {
        UpperEntry_OpenRemotePe4Output();
        UpperEntry_PrepareRemoteActions(UPPER_REMOTE_KEY_PD12);
        UpperEntry_ApplyRemotePd12First(tick_ms);
        upper_remote_auto_pd12_state =
            UPPER_REMOTE_AUTO_PD12_WAIT_FINAL_PE4;
    }

    if ((upper_remote_auto_pd11_state ==
         UPPER_REMOTE_AUTO_PD11_WAIT_SECOND_CLICK) &&
        ((int32_t)(tick_ms - upper_remote_auto_pd11_due_tick_ms) >= 0))
    {
        UpperEntry_StartRemoteAutoPd11SinglePress(tick_ms);
    }

    if ((upper_remote_auto_pd11_state ==
         UPPER_REMOTE_AUTO_PD11_WAIT_FIRST_DELAY) &&
        ((int32_t)(tick_ms - upper_remote_auto_pd11_due_tick_ms) >= 0))
    {
        UpperEntry_OpenRemotePe4Output();
        UpperEntry_ApplyRemoteM3508(UPPER_REMOTE_PD11_SECOND_M3508_DEG);
        if (upper_remote_mode == UPPER_REMOTE_MODE_STORE2_AUTO)
        {
            UpperEntry_ApplyRemoteJ4310(
                UPPER_REMOTE_STORE2_FINAL_J4310_DEG, tick_ms);
            upper_remote_auto_pd11_due_tick_ms = 0U;
            upper_remote_auto_pd11_state = UPPER_REMOTE_AUTO_PD11_IDLE;
        }
        else
        {
            upper_remote_auto_pd11_due_tick_ms =
                tick_ms + UPPER_REMOTE_PD11_J4310_DELAY_MS;
            upper_remote_auto_pd11_state =
                UPPER_REMOTE_AUTO_PD11_WAIT_J4310;
        }
    }
    else if ((upper_remote_auto_pd11_state ==
              UPPER_REMOTE_AUTO_PD11_WAIT_J4310) &&
             ((int32_t)(tick_ms - upper_remote_auto_pd11_due_tick_ms) >= 0))
    {
        UpperEntry_ApplyRemoteJ4310(
            UPPER_REMOTE_PD11_SECOND_J4310_DEG, tick_ms);
        upper_remote_auto_pd11_due_tick_ms =
            tick_ms + UPPER_REMOTE_PD11_CLOSE_DELAY_MS;
        upper_remote_auto_pd11_state =
            UPPER_REMOTE_AUTO_PD11_WAIT_FINAL_DELAY;
    }
    else if ((upper_remote_auto_pd11_state ==
              UPPER_REMOTE_AUTO_PD11_WAIT_FINAL_DELAY) &&
             ((int32_t)(tick_ms - upper_remote_auto_pd11_due_tick_ms) >= 0))
    {
        UpperEntry_CloseRemotePe4Output();
        upper_remote_auto_pd11_due_tick_ms =
            tick_ms + UPPER_REMOTE_PD11_J4310_AFTER_CLOSE_DELAY_MS;
        upper_remote_auto_pd11_state =
            UPPER_REMOTE_AUTO_PD11_WAIT_FINAL_J4310;
    }
    else if ((upper_remote_auto_pd11_state ==
              UPPER_REMOTE_AUTO_PD11_WAIT_FINAL_J4310) &&
             ((int32_t)(tick_ms - upper_remote_auto_pd11_due_tick_ms) >= 0))
    {
        UpperEntry_ApplyRemoteJ4310(
            UPPER_REMOTE_PD11_FIRST_J4310_DEG, tick_ms);
        upper_remote_auto_pd11_due_tick_ms = 0U;
        upper_remote_auto_pd11_state = UPPER_REMOTE_AUTO_PD11_IDLE;
    }
    else if ((upper_remote_auto_pd11_state ==
              UPPER_REMOTE_AUTO_PD11_WAIT_DOUBLE_RETURN) &&
             ((int32_t)(tick_ms - upper_remote_auto_pd11_due_tick_ms) >= 0))
    {
        UpperEntry_OpenRemotePe4Output();
        UpperEntry_ApplyRemoteArm(UPPER_REMOTE_PD11_SECOND_M3508_DEG,
                                  UPPER_REMOTE_STORE2_FINAL_J4310_DEG,
                                  tick_ms);
        UpperEntry_ResetRemoteAutoPd11Sequence();
    }
}
/* ==================== 遥控手动控制 ==================== */


/* 新机械臂动作会取消与其冲突的延时动作，并更新各按键的单双次状态。 */
static void UpperEntry_PrepareRemoteActions(uint8_t action_bits)
{
    if ((action_bits & (UPPER_REMOTE_KEY_PD13 |
                        UPPER_REMOTE_KEY_PD12 |
                        UPPER_REMOTE_KEY_PD11 |
                        UPPER_REMOTE_KEY_PD8)) != 0U)
    {
        upper_remote_pd11_pending = false;
        upper_remote_pd8_first_pending = false;
    }
    if ((action_bits & (UPPER_REMOTE_KEY_PD13 |
                        UPPER_REMOTE_KEY_PD12 |
                        UPPER_REMOTE_KEY_PD11 |
                        UPPER_REMOTE_KEY_PD10)) != 0U)
    {
        upper_remote_pd9_zero_pending = false;
    }
    if ((action_bits & UPPER_REMOTE_PD13_RESET_KEYS) != 0U)
    {
        upper_remote_pd13_second = false;
    }
    if ((action_bits & UPPER_REMOTE_PD12_RESET_KEYS) != 0U)
    {
        upper_remote_pd12_second = false;
    }
    if ((action_bits & UPPER_REMOTE_PD8_RESET_KEYS) != 0U)
    {
        upper_remote_pd8_second = false;
    }
    if ((action_bits & UPPER_REMOTE_PD11_RESET_KEYS) != 0U)
    {
        upper_remote_pd11_second = false;
        if (upper_remote_mode == UPPER_REMOTE_MODE_STORE2_AUTO)
        {
            UpperEntry_ResetRemoteAutoPd11Sequence();
        }
    }
}

/* 功能：执行 PD13 的第一组机械臂姿态；用途：完成对应阶段动作并将 PD13 状态切到第二组。 */
static void UpperEntry_ApplyRemotePd13First(uint32_t tick_ms)
{
    UpperEntry_ApplyRemoteArm(
        UPPER_REMOTE_PD13_FIRST_M3508_DEG,
        UPPER_REMOTE_PD13_FIRST_J4310_DEG,
        tick_ms);
    upper_remote_pd13_second = true;
}

/* 功能：执行 PD13 的第二组机械臂姿态；用途：完成对应阶段动作并将 PD13 状态切回第一组。 */
static void UpperEntry_ApplyRemotePd13Second(uint32_t tick_ms)
{
    UpperEntry_ApplyRemoteArm(
        UPPER_REMOTE_PD13_SECOND_M3508_DEG,
        UPPER_REMOTE_AUTO_PD13_SECOND_J4310_DEG,
        tick_ms);
    upper_remote_pd13_second = false;
}

/* 功能：处理遥控器 PD13 按键动作；用途：在 PD13 两组机械臂预设姿态之间交替切换。 */
static void UpperEntry_HandleRemotePd13(uint32_t tick_ms)
{
    if (upper_remote_pd13_second)
    {
        UpperEntry_ApplyRemoteArm(
            UPPER_REMOTE_PD13_SECOND_M3508_DEG,
            UPPER_REMOTE_PD13_SECOND_J4310_DEG,
            tick_ms);
        upper_remote_pd13_second = false;
    }
    else
    {
        UpperEntry_ApplyRemoteArm(
            UPPER_REMOTE_PD13_FIRST_M3508_DEG,
            UPPER_REMOTE_PD13_FIRST_J4310_DEG,
            tick_ms);
        upper_remote_pd13_second = true;
    }
}

/* 功能：执行 PD12 的第一组机械臂姿态；用途：下发首段目标并将下一次动作切换到第二组。 */
static void UpperEntry_ApplyRemotePd12First(uint32_t tick_ms)
{
    UpperEntry_ApplyRemoteArm(
        UPPER_REMOTE_PD12_FIRST_M3508_DEG,
        UPPER_REMOTE_PD12_FIRST_J4310_DEG,
        tick_ms);
    upper_remote_pd12_second = true;
}

/* 功能：执行 PD12 的第二组机械臂姿态；用途：下发次段目标并将下一次动作切回第一组。 */
static void UpperEntry_ApplyRemotePd12Second(uint32_t tick_ms)
{
    UpperEntry_ApplyRemoteArm(
        UPPER_REMOTE_PD12_SECOND_M3508_DEG,
        UPPER_REMOTE_PD12_SECOND_J4310_DEG,
        tick_ms);
    upper_remote_pd12_second = false;
}

/* 功能：处理遥控器 PD12 按键动作；用途：根据当前阶段选择并执行第一或第二组预设姿态。 */
static void UpperEntry_HandleRemotePd12(uint32_t tick_ms)
{
    if (upper_remote_pd12_second)
    {
        UpperEntry_ApplyRemotePd12Second(tick_ms);
    }
    else
    {
        UpperEntry_ApplyRemotePd12First(tick_ms);
    }
}

/* 功能：启动 PD11 分步机械臂动作；用途：先设置 M3508 角度，再登记 500 ms 后执行的 J4310 目标。 */
static void UpperEntry_StartRemotePd11(float m3508_angle_deg,
                                       float j4310_angle_deg,
                                       uint32_t tick_ms)
{
    UpperEntry_ApplyRemoteM3508(m3508_angle_deg);
    upper_remote_pd11_pending = true;
    upper_remote_pd11_due_tick_ms = tick_ms + 500U;
    upper_remote_pd11_pending_j4310_deg = j4310_angle_deg;
}

/* 功能：启动 PD11 的第一组分步动作；用途：使用第一组 M3508 和 J4310 预设角度建立延时任务。 */
static void UpperEntry_StartRemotePd11First(uint32_t tick_ms)
{
    UpperEntry_StartRemotePd11(UPPER_REMOTE_PD11_FIRST_M3508_DEG,
                               UPPER_REMOTE_PD11_FIRST_J4310_DEG,
                               tick_ms);
}

/* 功能：处理遥控器 PD11 按键动作；用途：在两组分步姿态间切换并更新单双次状态。 */
static void UpperEntry_HandleRemotePd11(uint32_t tick_ms)
{
    if (upper_remote_pd11_second)
    {
        UpperEntry_StartRemotePd11(UPPER_REMOTE_PD11_SECOND_M3508_DEG,
                                   UPPER_REMOTE_PD11_SECOND_J4310_DEG,
                                   tick_ms);
    }
    else
    {
        UpperEntry_StartRemotePd11First(tick_ms);
    }
    upper_remote_pd11_second = !upper_remote_pd11_second;
}

/* 功能：执行 PD8 的第二组机械臂姿态；用途：取消 PD9 归零请求、恢复其切换状态并完成第二段动作。 */
static void UpperEntry_ApplyRemotePd8Second(float j4310_angle_deg,
                                             uint32_t tick_ms)
{
    upper_remote_pd9_zero_pending = false;
    upper_remote_pd9_second = false;
    UpperEntry_ApplyRemoteArm(
        UPPER_REMOTE_PD8_SECOND_M3508_DEG,
        j4310_angle_deg,
        tick_ms);
    upper_remote_pd8_second = false;
}

/* 功能：处理遥控器 PD8 按键动作；用途：首按启动延时分步动作，次按直接执行第二组姿态。 */
static void UpperEntry_HandleRemotePd8(uint32_t tick_ms)
{
    if (upper_remote_pd8_second)
    {
        UpperEntry_ApplyRemotePd8Second(
            UPPER_REMOTE_PD8_SECOND_J4310_DEG, tick_ms);
    }
    else
    {
        UpperEntry_ApplyRemoteM3508(UPPER_REMOTE_PD8_FIRST_M3508_DEG);
        upper_remote_pd8_first_pending = true;
        upper_remote_pd8_first_due_tick_ms = tick_ms + 500U;
        upper_remote_pd9_zero_pending = true;
        upper_remote_pd8_second = true;
    }
}

/* 功能：处理遥控器 PD9 按键动作；用途：优先执行待处理的闸门归零，否则在两组闸门角度间切换。 */
static void UpperEntry_HandleRemotePd9(void)
{
    if (upper_remote_pd9_zero_pending)
    {
        UpperEntry_ApplyRemoteGate(UPPER_REMOTE_PD9_ZERO_GATE_DEG);
        upper_remote_pd9_zero_pending = false;
        upper_remote_pd9_second = false;
    }
    else
    {
        UpperEntry_ApplyRemoteGate(
            upper_remote_pd9_second ?
                UPPER_REMOTE_PD9_SECOND_GATE_DEG :
                UPPER_REMOTE_PD9_FIRST_GATE_DEG);
        upper_remote_pd9_second = !upper_remote_pd9_second;
    }
}

/* 功能：处理遥控器 PD10 按键动作；用途：在夹爪的第一、第二目标角度之间交替切换。 */
static void UpperEntry_HandleRemotePd10(void)
{
    UpperEntry_ApplyRemoteGripper(
        upper_remote_pd10_second ?
            UPPER_REMOTE_PD10_SECOND_GRIPPER_DEG :
            UPPER_REMOTE_PD10_FIRST_GRIPPER_DEG,
        true);
    upper_remote_pd10_second = !upper_remote_pd10_second;
}

/* PD9、PD10 独立于自动存取状态机，PC0 等待期间也必须响应。 */
static void UpperEntry_HandleRemotePd9Pd10(uint8_t rising_bits)
{
    UpperEntry_PrepareRemoteActions(
        rising_bits & (UPPER_REMOTE_KEY_PD9 | UPPER_REMOTE_KEY_PD10));
    if ((rising_bits & UPPER_REMOTE_KEY_PD9) != 0U)
    {
        UpperEntry_HandleRemotePd9();
    }
    if ((rising_bits & UPPER_REMOTE_KEY_PD10) != 0U)
    {
        UpperEntry_HandleRemotePd10();
    }
}
