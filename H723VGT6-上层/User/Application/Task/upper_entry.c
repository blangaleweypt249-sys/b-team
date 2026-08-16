#include "upper_entry.h"

#include <string.h>

#include "can_id.h"
#include "cmsis_os2.h"
#include "comm_runtime.h"
#include "j4310_auto_return.h"
#include "j4310_position_control.h"
#include "upper_config.h"
#include "upper_motor_port.h"
#include "upper_pc_link.h"
#include "upper_remote_link.h"
#include "W25Qxx.h"

#define UPPER_CMD_QUEUE_DEPTH  4U
#define UPPER_STATE_PERIOD_MS  50U
#define UPPER_DJI_TELEMETRY_PERIOD_MS 10U
#define UPPER_TX_BUFFER_SIZE   160U
#define UPPER_HANDSHAKE_ACK_GUARD_MS 20U
#define UPPER_J4310_AUTO_RETURN_KP 5.0f
#define UPPER_J4310_AUTO_RETURN_KD 0.5f
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

#define UPPER_REMOTE_PD13_RESET_KEYS \
    (UPPER_REMOTE_KEY_PD12 | UPPER_REMOTE_KEY_PD11 | \
     UPPER_REMOTE_KEY_PD8)
#define UPPER_REMOTE_PD12_RESET_KEYS \
    (UPPER_REMOTE_KEY_PD13 | UPPER_REMOTE_KEY_PD11 | \
     UPPER_REMOTE_KEY_PD8)
#define UPPER_REMOTE_PD11_RESET_KEYS \
    (UPPER_REMOTE_KEY_PD13 | UPPER_REMOTE_KEY_PD12 | \
     UPPER_REMOTE_KEY_PD8)

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
static bool upper_remote_pd13_reset_pending;
static uint32_t upper_remote_pd13_reset_due_tick_ms;
static bool upper_remote_pd12_second;
static bool upper_remote_pd11_second;
static uint8_t upper_remote_pd8_stage;
static bool upper_remote_pd8_first_pending;
static uint32_t upper_remote_pd8_first_due_tick_ms;
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

static void UpperEntry_OnPcAuxControl(uint8_t output_bits, void *user_data)
{
    (void)user_data;
    upper_aux_output_bits = output_bits & 0x0FU;
    upper_aux_spi3_pending = true;
}

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
    /* UART4 is the PC link; UART5 carries the fixed 10-byte remote frame. */
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

static void UpperEntry_CancelJ4310AutoReturn(void)
{
    J4310AutoReturn_Cancel(&upper_j4310_auto_return);
    (void)MotorManager_ClearOverride(&upper_robot.motor_manager,
                                     UPPER_MOTOR_ARM_J4310);
}

static bool UpperEntry_ResetJ4310PositionControl(void)
{
    return J4310PositionControl_Init(
        &upper_j4310_position_control,
        UPPER_J4310_TRAJECTORY_MAX_VEL_RAD_S,
        UPPER_J4310_TRAJECTORY_MAX_ACCEL_RAD_S2,
        UPPER_J4310_GRAVITY_MODEL_LIMIT_NM,
        UPPER_J4310_GRAVITY_LEARNING_RATE);
}

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
            target.arm.grip_kp = UPPER_J4310_AUTO_RETURN_KP;
            target.arm.grip_kd = UPPER_J4310_AUTO_RETURN_KD;
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
        command.kp = UPPER_J4310_AUTO_RETURN_KP;
        command.kd = UPPER_J4310_AUTO_RETURN_KD;
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

static float UpperEntry_RemoteDegreesToRadians(float degrees)
{
    return degrees * UPPER_REMOTE_ANGLE_DEG_TO_RAD;
}

/* Apply one complete arm target so the two M3508s and J4310 move together. */
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
    target.arm.grip_kp = UPPER_J4310_AUTO_RETURN_KP;
    target.arm.grip_kd = UPPER_J4310_AUTO_RETURN_KD;
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

/* Hold the current J4310 target while moving only the two M3508s. */
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

/* Keep the M3508 target unchanged while moving only J4310. */
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
    target.arm.grip_kp = UPPER_J4310_AUTO_RETURN_KP;
    target.arm.grip_kd = UPPER_J4310_AUTO_RETURN_KD;
    target.arm.grip_torque_nm = 0.0f;
    target.arm.grip_torque_limit_nm = UPPER_J4310_TORQUE_MAP_MAX_NM;

    upper_j4310_startup_enable_pending = false;
    UpperEntry_CancelJ4310AutoReturn();
    UpperEntry_StartJ4310Trajectory(tick_ms, target.arm.grip_pos_rad);
    UpperRobot_SetTarget(&upper_robot, &target);
    UpperRobot_Start(&upper_robot);
}

static void UpperEntry_ApplyRemoteGate(float angle_deg)
{
    upper_target_t target;

    target = upper_robot.target;
    target.position_mode = true;
    target.conveyor.enabled = true;
    target.conveyor.position_mode = true;
    target.conveyor.pid_update = false;
    target.conveyor.m2006_pos_rad =
        UpperEntry_RemoteDegreesToRadians(angle_deg);
    UpperRobot_SetTarget(&upper_robot, &target);
    UpperRobot_Start(&upper_robot);
}

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

/* Consume only rising edges; the received remote frame is a level snapshot. */
/* Consume rising edges from the UART5 fixed-frame remote level snapshot. */
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
        upper_remote_pd13_reset_pending = false;
        upper_remote_pd8_first_pending = false;
    }
    else
    {
        key_bits = control.key_bits & UPPER_REMOTE_KEY_MASK;
    }
    rising_bits = key_bits & (uint8_t)~upper_remote_previous_key_bits;
    upper_remote_previous_key_bits = key_bits;
    if (rising_bits != 0U)
    {
        /* A fresh action supersedes any delayed remote action. */
        upper_remote_pd13_reset_pending = false;
        upper_remote_pd8_first_pending = false;
    }

    if ((rising_bits & UPPER_REMOTE_PD13_RESET_KEYS) != 0U)
    {
        upper_remote_pd13_second = false;
    }
    if ((rising_bits & UPPER_REMOTE_PD12_RESET_KEYS) != 0U)
    {
        upper_remote_pd12_second = false;
    }
    if ((rising_bits & UPPER_REMOTE_PD11_RESET_KEYS) != 0U)
    {
        upper_remote_pd11_second = false;
    }
    if ((rising_bits & (UPPER_REMOTE_KEY_PD13 |
                       UPPER_REMOTE_KEY_PD12 |
                       UPPER_REMOTE_KEY_PD11)) != 0U)
    {
        upper_remote_pd8_stage = 0U;
    }

    if ((rising_bits & UPPER_REMOTE_KEY_PD13) != 0U)
    {
        if (upper_remote_pd13_second)
        {
            UpperEntry_ApplyRemoteM3508(
                UPPER_REMOTE_PD13_SECOND_M3508_DEG);
            upper_remote_pd13_reset_pending = true;
            upper_remote_pd13_reset_due_tick_ms =
                tick_ms + UPPER_REMOTE_PD13_RESET_DELAY_MS;
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
        if (upper_remote_pd11_second)
        {
            UpperEntry_ApplyRemoteArm(
                UPPER_REMOTE_PD11_SECOND_M3508_DEG,
                UPPER_REMOTE_PD11_SECOND_J4310_DEG,
                tick_ms);
            upper_remote_pd11_second = false;
        }
        else
        {
            UpperEntry_ApplyRemoteArm(
                UPPER_REMOTE_PD11_FIRST_M3508_DEG,
                UPPER_REMOTE_PD11_FIRST_J4310_DEG,
                tick_ms);
            upper_remote_pd11_second = true;
        }
    }
    if ((rising_bits & UPPER_REMOTE_KEY_PD8) != 0U)
    {
        if (upper_remote_pd8_stage == 0U)
        {
            UpperEntry_ApplyRemoteM3508(
                UPPER_REMOTE_PD8_FIRST_M3508_DEG);
            upper_remote_pd8_first_pending = true;
            upper_remote_pd8_first_due_tick_ms =
                tick_ms + UPPER_REMOTE_PD8_FIRST_DELAY_MS;
            upper_remote_pd8_stage = 1U;
        }
        else if (upper_remote_pd8_stage == 1U)
        {
            UpperEntry_ApplyRemoteArm(
                UPPER_REMOTE_PD8_SECOND_M3508_DEG,
                UPPER_REMOTE_PD8_SECOND_J4310_DEG,
                tick_ms);
            upper_remote_pd8_stage = 2U;
        }
        else
        {
            UpperEntry_ApplyRemoteArm(
                UPPER_REMOTE_PD8_THIRD_M3508_DEG,
                UPPER_REMOTE_PD8_THIRD_J4310_DEG,
                tick_ms);
            upper_remote_pd8_stage = 0U;
        }
    }
    if ((rising_bits & UPPER_REMOTE_KEY_PD9) != 0U)
    {
        UpperEntry_ApplyRemoteGate(
            upper_remote_pd9_second ?
                UPPER_REMOTE_PD9_SECOND_GATE_DEG :
                UPPER_REMOTE_PD9_FIRST_GATE_DEG);
        upper_remote_pd9_second = !upper_remote_pd9_second;
    }
    if ((rising_bits & UPPER_REMOTE_KEY_PD10) != 0U)
    {
        UpperEntry_ApplyRemoteGripper(
            upper_remote_pd10_second ?
                UPPER_REMOTE_PD10_SECOND_GRIPPER_DEG :
                UPPER_REMOTE_PD10_FIRST_GRIPPER_DEG);
        upper_remote_pd10_second = !upper_remote_pd10_second;
    }

    /* A different edge in the same frame supersedes the delayed reset. */
    if ((rising_bits & (UPPER_REMOTE_KEY_PD12 |
                       UPPER_REMOTE_KEY_PD11 |
                       UPPER_REMOTE_KEY_PD8 |
                       UPPER_REMOTE_KEY_PD9 |
                       UPPER_REMOTE_KEY_PD10)) != 0U)
    {
        upper_remote_pd13_reset_pending = false;
    }
    if ((rising_bits & (UPPER_REMOTE_KEY_PD13 |
                       UPPER_REMOTE_KEY_PD12 |
                       UPPER_REMOTE_KEY_PD11 |
                       UPPER_REMOTE_KEY_PD9 |
                       UPPER_REMOTE_KEY_PD10)) != 0U)
    {
        upper_remote_pd8_first_pending = false;
    }

    if (upper_remote_pd13_reset_pending &&
        ((int32_t)(tick_ms - upper_remote_pd13_reset_due_tick_ms) >= 0))
    {
        UpperEntry_ApplyRemoteJ4310(
            UPPER_REMOTE_PD13_SECOND_J4310_DEG, tick_ms);
        upper_remote_pd13_reset_pending = false;
    }
    if (upper_remote_pd8_first_pending &&
        ((int32_t)(tick_ms - upper_remote_pd8_first_due_tick_ms) >= 0))
    {
        UpperEntry_ApplyRemoteJ4310(
            UPPER_REMOTE_PD8_FIRST_J4310_DEG, tick_ms);
        upper_remote_pd8_first_pending = false;
    }
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
    upper_remote_pd13_reset_pending = false;
    upper_remote_pd13_reset_due_tick_ms = 0U;
    upper_remote_pd12_second = false;
    upper_remote_pd11_second = false;
    upper_remote_pd8_stage = 0U;
    upper_remote_pd8_first_pending = false;
    upper_remote_pd8_first_due_tick_ms = 0U;
    upper_remote_pd9_second = false;
    upper_remote_pd10_second = false;
    /* Send an initial zero state and keep repeating it until the receiver is up. */
    upper_aux_spi3_pending = true;
    upper_aux_output_bits = 0U;
    upper_aux_spi3_sequence = 0U;
    upper_aux_spi3_last_sent_tick_ms = 0U;
    upper_aux_spi3_have_sent = false;
    upper_aux_spi3_sent_count = 0U;
    upper_aux_spi3_fail_count = 0U;
    CommRuntime_SetHandlers(UpperEntry_OnUart, UpperEntry_OnCan, NULL);
    robot_ready = UpperRobot_Init(&upper_robot, UpperMotorPort_Send, NULL);
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
    j4310_auto_return_status.available = true;
    j4310_auto_return_status.enabled =
        upper_j4310_auto_return_enabled;
    j4310_auto_return_status.active =
        J4310AutoReturn_IsActive(&upper_j4310_auto_return);
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
    UpperEntry_ProcessCmd(tick_ms);
    UpperEntry_ProcessRemote(tick_ms);
    UpperEntry_ProcessAuxSpi3(tick_ms);
    UpperEntry_ProcessMotorAction(tick_ms);
    if (upper_estop_pending)
    {
        upper_estop_pending = false;
        upper_remote_pd13_reset_pending = false;
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
