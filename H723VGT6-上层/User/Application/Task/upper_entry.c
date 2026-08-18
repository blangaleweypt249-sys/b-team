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
#define UPPER_REMOTE_ANGLE_DEG_TO_RAD 0.017453292519943295f
#define UPPER_REMOTE_GRIPPER_MOTOR_DEG_PER_OUTPUT_DEG 2.0f
#define UPPER_AUX_SPI_FRAME_SIZE       8U
#define UPPER_AUX_SPI_TARGET_RECEIVER  0x01U
#define UPPER_AUX_SPI_TYPE_CONTROL     0x02U
#define UPPER_AUX_SPI_PAYLOAD_SIZE     1U
#define UPPER_AUX_SPI_REPEAT_PERIOD_MS 50U
#define UPPER_CONTROL_PERIOD_MS          1U

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
static uint8_t upper_remote_previous_key_bits;
static bool upper_remote_pd13_second;
static bool upper_remote_pd12_second;
static bool upper_remote_pd11_first_pending;
static uint32_t upper_remote_pd11_first_due_tick_ms;
static bool upper_remote_pd8_second;
static bool upper_remote_pd8_first_pending;
static uint32_t upper_remote_pd8_first_due_tick_ms;
static bool upper_remote_pd9_zero_pending;
static bool upper_remote_pd9_oscillation_pending;
static bool upper_remote_pd9_gate_oscillating;
static bool upper_remote_pd9_gate_high_next;
static uint32_t upper_remote_pd9_gate_due_tick_ms;
static bool upper_remote_pd9_second;
static bool upper_remote_pd10_second;
static volatile bool upper_aux_spi3_pending;
static volatile uint8_t upper_aux_output_bits;
static uint8_t upper_aux_spi3_sequence;
static uint8_t upper_aux_spi3_frame[UPPER_AUX_SPI_FRAME_SIZE];
static uint32_t upper_aux_spi3_last_sent_tick_ms;
static bool upper_aux_spi3_have_sent;
volatile uint32_t upper_handshake_ack_sent_count;
volatile uint32_t upper_handshake_ack_busy_count;
volatile uint32_t upper_handshake_ack_fail_count;
volatile uint32_t upper_state_sent_count;
volatile uint32_t upper_state_busy_count;
volatile uint32_t upper_state_fail_count;
volatile uint32_t upper_dji_telemetry_sent_count;
volatile uint32_t upper_dji_telemetry_busy_count;
volatile uint32_t upper_dji_telemetry_fail_count;
volatile uint32_t upper_aux_spi3_sent_count;
volatile uint32_t upper_aux_spi3_fail_count;
__ALIGNED(32) static uint8_t upper_tx_buffer[UPPER_TX_BUFFER_SIZE];

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

/* 功能：处理 PC 下发的辅助输出位；用途：更新可经 SPI 转发的辅助控制状态；无返回值表示最新位状态已保存。 */
static void UpperEntry_OnPcAuxControl(uint8_t output_bits, void *user_data)
{
    (void)user_data;
    upper_aux_output_bits = output_bits & 0x0FU;
    upper_aux_spi3_pending = true;
}

/* 功能：计算字节序列的 CRC8 校验值；用途：保护辅助 SPI3 数据帧；返回值表示 CRC8 结果。 */
static uint8_t UpperEntry_Crc8(const uint8_t *data, size_t size)
{
    uint8_t crc;
    size_t index;
    uint8_t bit;

    crc = 0U;
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

/* 功能：周期构造并发送辅助 SPI3 控制帧；用途：把 PC 辅助输出状态转发给外部设备；无返回值表示本周期发送已处理。 */
static void UpperEntry_ProcessAuxSpi3(uint32_t tick_ms)
{
    uint8_t output_bits;

    if (!upper_aux_spi3_pending &&
        upper_aux_spi3_have_sent &&
        ((tick_ms - upper_aux_spi3_last_sent_tick_ms) <
         UPPER_AUX_SPI_REPEAT_PERIOD_MS))
    {
        return;
    }
    if (!CommRuntime_Spi3TxReady())
    {
        return;
    }
    output_bits = upper_aux_output_bits;
    upper_aux_spi3_frame[0] = 0xA5U;
    upper_aux_spi3_frame[1] = 0x5AU;
    upper_aux_spi3_frame[2] = UPPER_AUX_SPI_TARGET_RECEIVER;
    upper_aux_spi3_frame[3] = UPPER_AUX_SPI_TYPE_CONTROL;
    upper_aux_spi3_frame[4] = UPPER_AUX_SPI_PAYLOAD_SIZE;
    upper_aux_spi3_frame[5] = upper_aux_spi3_sequence++;
    upper_aux_spi3_frame[6] = output_bits;
    upper_aux_spi3_frame[7] = UpperEntry_Crc8(
                                   &upper_aux_spi3_frame[2],
                                   UPPER_AUX_SPI_FRAME_SIZE - 3U);
    if (CommRuntime_Spi3Transmit(upper_aux_spi3_frame,
                                 UPPER_AUX_SPI_FRAME_SIZE))
    {
        if (upper_aux_output_bits == output_bits)
        {
            upper_aux_spi3_pending = false;
        }
        upper_aux_spi3_last_sent_tick_ms = tick_ms;
        upper_aux_spi3_have_sent = true;
        upper_aux_spi3_sent_count++;
    }
    else
    {
        upper_aux_spi3_fail_count++;
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
    UpperRobot_SetTarget(&upper_robot, &target);
    UpperRobot_Start(&upper_robot);
}

/* 功能：应用遥控器给出的闸门角度目标；用途：更新闸门位置和使能状态；无返回值表示目标已提交。 */
static void UpperEntry_ApplyRemoteGate(float angle_deg)
{
    upper_target_t target;

    target = upper_robot.target;
    target.position_mode = true;
    target.gate.enabled = true;
    target.gate.position_mode = true;
    target.gate.pid_update = false;
    target.gate.m2006_pos_rad =
        UpperEntry_RemoteDegreesToRadians(angle_deg);
    UpperRobot_SetTarget(&upper_robot, &target);
    UpperRobot_Start(&upper_robot);
}

/* 功能：应用遥控器给出的夹爪角度目标；用途：更新夹爪位置和使能状态；无返回值表示目标已提交。 */
static void UpperEntry_ApplyRemoteGripper(float angle_deg)
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
    UpperRobot_SetTarget(&upper_robot, &target);
    UpperRobot_Start(&upper_robot);
}

/* 功能：处理遥控器 PD9 输入对应的闸门动作；用途：根据电平和消抖状态更新闸门目标；无返回值表示本周期输入已处理。 */
static void UpperEntry_ProcessRemotePd9Gate(uint32_t tick_ms)
{
    float angle_deg;

    if (!upper_remote_pd9_gate_oscillating ||
        ((int32_t)(tick_ms - upper_remote_pd9_gate_due_tick_ms) < 0))
    {
        return;
    }
    angle_deg = upper_remote_pd9_gate_high_next ?
                UPPER_REMOTE_PD9_OSCILLATION_HIGH_DEG :
                UPPER_REMOTE_PD9_OSCILLATION_LOW_DEG;
    UpperEntry_ApplyRemoteGate(angle_deg);
    upper_remote_pd9_gate_high_next =
        !upper_remote_pd9_gate_high_next;
    upper_remote_pd9_gate_due_tick_ms += 500U;
}

/* 只处理上升沿；接收到的遥控帧是电平快照。 */
/* 从 UART5 固定帧遥控电平快照中处理上升沿。 */
/* 功能：解析遥控按键和摇杆并更新机器人目标；用途：实现遥控动作映射、边沿触发和超时处理；无返回值表示本周期遥控输入已处理。 */
static void UpperEntry_ProcessRemote(uint32_t tick_ms)
{
    upper_remote_control_t control;
    bool remote_online;
    uint8_t key_bits;
    uint8_t rising_bits;

    remote_online = UpperEntry_GetSecondaryRemoteControl(&control, tick_ms) &&
                    control.online;
    if (!remote_online)
    {
        key_bits = 0U;
        upper_remote_pd11_first_pending = false;
        upper_remote_pd8_first_pending = false;
        upper_remote_pd9_zero_pending = false;
        upper_remote_pd9_oscillation_pending = false;
        upper_remote_pd9_gate_oscillating = false;
    }
    else
    {
        key_bits = control.key_bits & UPPER_REMOTE_KEY_MASK;
    }
    rising_bits = key_bits & (uint8_t)~upper_remote_previous_key_bits;
    upper_remote_previous_key_bits = key_bits;
    if ((rising_bits & (UPPER_REMOTE_KEY_PD13 |
                       UPPER_REMOTE_KEY_PD12 |
                       UPPER_REMOTE_KEY_PD11 |
                       UPPER_REMOTE_KEY_PD8)) != 0U)
    {
        upper_remote_pd11_first_pending = false;
        upper_remote_pd8_first_pending = false;
        upper_remote_pd9_gate_oscillating = false;
    }
    if ((rising_bits & (UPPER_REMOTE_KEY_PD13 |
                       UPPER_REMOTE_KEY_PD12 |
                       UPPER_REMOTE_KEY_PD11 |
                       UPPER_REMOTE_KEY_PD10)) != 0U)
    {
        upper_remote_pd9_zero_pending = false;
        upper_remote_pd9_oscillation_pending = false;
    }
    if ((rising_bits & UPPER_REMOTE_PD13_RESET_KEYS) != 0U)
    {
        upper_remote_pd13_second = false;
    }
    if ((rising_bits & UPPER_REMOTE_PD12_RESET_KEYS) != 0U)
    {
        upper_remote_pd12_second = false;
    }
    if ((rising_bits & UPPER_REMOTE_PD8_RESET_KEYS) != 0U)
    {
        upper_remote_pd8_second = false;
    }

    if ((rising_bits & UPPER_REMOTE_KEY_PD13) != 0U)
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
    if ((rising_bits & UPPER_REMOTE_KEY_PD12) != 0U)
    {
        if (upper_remote_pd12_second)
        {
            UpperEntry_ApplyRemoteArm(
                UPPER_REMOTE_PD12_SECOND_M3508_DEG,
                UPPER_REMOTE_PD12_SECOND_J4310_DEG,
                tick_ms);
            upper_remote_pd12_second = false;
        }
        else
        {
            UpperEntry_ApplyRemoteArm(
                UPPER_REMOTE_PD12_FIRST_M3508_DEG,
                UPPER_REMOTE_PD12_FIRST_J4310_DEG,
                tick_ms);
            upper_remote_pd12_second = true;
        }
    }
    if ((rising_bits & UPPER_REMOTE_KEY_PD11) != 0U)
    {
        UpperEntry_ApplyRemoteM3508(
            UPPER_REMOTE_PD11_FIRST_M3508_DEG);
        upper_remote_pd11_first_pending = true;
        upper_remote_pd11_first_due_tick_ms =
            tick_ms + 500U;
    }
    if ((rising_bits & UPPER_REMOTE_KEY_PD8) != 0U)
    {
        if (upper_remote_pd8_second)
        {
            upper_remote_pd9_zero_pending = false;
            upper_remote_pd9_oscillation_pending = true;
            upper_remote_pd9_gate_oscillating = false;
            upper_remote_pd9_second = false;
            UpperEntry_ApplyRemoteArm(
                UPPER_REMOTE_PD8_SECOND_M3508_DEG,
                UPPER_REMOTE_PD8_SECOND_J4310_DEG,
                tick_ms);
            upper_remote_pd8_second = false;
        }
        else
        {
            UpperEntry_ApplyRemoteM3508(
                UPPER_REMOTE_PD8_FIRST_M3508_DEG);
            upper_remote_pd8_first_pending = true;
            upper_remote_pd8_first_due_tick_ms =
                tick_ms + 500U;
            upper_remote_pd9_zero_pending = true;
            upper_remote_pd9_oscillation_pending = false;
            upper_remote_pd9_gate_oscillating = false;
            upper_remote_pd8_second = true;
        }
    }
    if ((rising_bits & UPPER_REMOTE_KEY_PD9) != 0U)
    {
        upper_remote_pd9_gate_oscillating = false;
        if (upper_remote_pd9_zero_pending)
        {
            UpperEntry_ApplyRemoteGate(UPPER_REMOTE_PD9_ZERO_GATE_DEG);
            upper_remote_pd9_zero_pending = false;
            upper_remote_pd9_oscillation_pending = false;
            upper_remote_pd9_second = false;
        }
        else if (upper_remote_pd9_oscillation_pending)
        {
            UpperEntry_ApplyRemoteGate(
                UPPER_REMOTE_PD9_OSCILLATION_HIGH_DEG);
            upper_remote_pd9_oscillation_pending = false;
            upper_remote_pd9_gate_oscillating = true;
            upper_remote_pd9_gate_high_next = false;
            upper_remote_pd9_gate_due_tick_ms =
                tick_ms + 500U;
            upper_remote_pd9_second = true;
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
    if ((rising_bits & UPPER_REMOTE_KEY_PD10) != 0U)
    {
        UpperEntry_ApplyRemoteGripper(
            upper_remote_pd10_second ?
                UPPER_REMOTE_PD10_SECOND_GRIPPER_DEG :
                UPPER_REMOTE_PD10_FIRST_GRIPPER_DEG);
        upper_remote_pd10_second = !upper_remote_pd10_second;
    }
    if (upper_remote_pd11_first_pending &&
        ((int32_t)(tick_ms - upper_remote_pd11_first_due_tick_ms) >= 0))
    {
        UpperEntry_ApplyRemoteJ4310(
            UPPER_REMOTE_PD11_FIRST_J4310_DEG, tick_ms);
        upper_remote_pd11_first_pending = false;
    }
    if (upper_remote_pd8_first_pending &&
        ((int32_t)(tick_ms - upper_remote_pd8_first_due_tick_ms) >= 0))
    {
        UpperEntry_ApplyRemoteJ4310(
            UPPER_REMOTE_PD8_FIRST_J4310_DEG, tick_ms);
        upper_remote_pd8_first_pending = false;
    }
    UpperEntry_ProcessRemotePd9Gate(tick_ms);
}

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
    upper_j4310_auto_return_enabled = false;
    upper_j4310_startup_enable_pending = true;
    upper_j4310_startup_enable_attempted = false;
    upper_j4310_startup_enable_last_tick_ms = 0U;
    upper_remote_previous_key_bits = 0U;
    upper_remote_pd13_second = false;
    upper_remote_pd12_second = false;
    upper_remote_pd11_first_pending = false;
    upper_remote_pd11_first_due_tick_ms = 0U;
    upper_remote_pd8_second = false;
    upper_remote_pd8_first_pending = false;
    upper_remote_pd8_first_due_tick_ms = 0U;
    upper_remote_pd9_zero_pending = false;
    upper_remote_pd9_oscillation_pending = false;
    upper_remote_pd9_gate_oscillating = false;
    upper_remote_pd9_gate_high_next = false;
    upper_remote_pd9_gate_due_tick_ms = 0U;
    upper_remote_pd9_second = false;
    upper_remote_pd10_second = false;
    /* 发送初始零状态，并在接收端就绪前持续重复发送。 */
    upper_aux_spi3_pending = true;
    upper_aux_output_bits = 0U;
    upper_aux_spi3_sequence = 0U;
    upper_aux_spi3_last_sent_tick_ms = 0U;
    upper_aux_spi3_have_sent = false;
    upper_aux_spi3_sent_count = 0U;
    upper_aux_spi3_fail_count = 0U;
    CommRuntime_SetHandlers(UpperEntry_OnUart, UpperEntry_OnCan, NULL);
    robot_ready = UpperRobot_Init(&upper_robot,
                                  upper_motor_cfg,
                                  UPPER_MOTOR_COUNT,
                                  UpperMotorPort_Send,
                                  NULL);
    return robot_ready;
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
        if (j4310_commanded)
        {
            upper_j4310_startup_enable_pending = false;
            UpperEntry_CancelJ4310AutoReturn();
            if (target.arm.enabled)
            {
                UpperEntry_StartJ4310Trajectory(
                    tick_ms, target.arm.grip_pos_rad);
            }
            else
            {
                J4310PositionControl_CancelTrajectory(
                    &upper_j4310_position_control);
            }
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

/* 功能：按周期发送机器人状态；用途：向上位机报告运行态、通信统计和关节位置；无返回值表示结果写入发送统计。 */
static void UpperEntry_SendState(uint32_t tick_ms)
{
    size_t frame_size;
    upper_motor_fault_t fault;
    upper_j4310_rx_diagnostic_t j4310_rx_diagnostic;
    upper_j4310_tx_diagnostic_t j4310_tx_diagnostic;
    upper_pc_state_t pc_state;
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
    pc_state.j4310_auto_return.enabled =
        upper_j4310_auto_return_enabled;
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

/* 功能：轮询发送 DJI 电机诊断遥测；用途：分时上报多台电机与 FDCAN 接收计数；无返回值表示推进遥测索引。 */
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

/* 功能：执行上层应用的 1 ms 主控制周期；用途：处理命令、控制电机、检查故障并发送状态；无返回值表示完成一次调度。 */
void UpperEntry_Control1ms(uint32_t tick_ms)
{
    UpperMotorPort_BeginCycle(tick_ms);
    UpperEntry_ProcessCmd(tick_ms);
    UpperEntry_ProcessRemote(tick_ms);
    UpperEntry_ProcessAuxSpi3(tick_ms);
    UpperEntry_ProcessMotorAction(tick_ms);
    if (upper_estop_pending)
    {
        upper_estop_pending = false;
        upper_remote_pd11_first_pending = false;
        upper_remote_pd8_first_pending = false;
        upper_j4310_startup_enable_pending = false;
        J4310AutoReturn_Cancel(&upper_j4310_auto_return);
        J4310PositionControl_CancelTrajectory(
            &upper_j4310_position_control);
        UpperRobot_EStop(&upper_robot);
    }

    UpperEntry_ServiceJ4310StartupEnable(tick_ms);
    UpperEntry_ServiceJ4310Control(tick_ms);
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

/* 功能：使用 HAL 毫秒时基调用上层控制周期；用途：提供给系统任务的固定入口；无返回值表示完成本次 1 ms 调用。 */
void App_Control1ms(void)
{
    UpperEntry_Control1ms(CommRuntime_GetTickMs());
}
