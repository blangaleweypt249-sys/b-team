/**
 * @file test_upper_motor_routing.c
 * @brief 验证上层电机端口的型号路由、限位和诊断行为。
 */

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "bsp_can.h"
#include "DM J4310/j4310.h"
#include "M2006/m2006.h"
#include "M3508/m3508.h"
#include "upper_motor_port.h"

#define TEST_TX_CAPACITY  16U
#define TEST_DJI_ZERO_STABLE_FRAMES 5U
#define TEST_PI 3.14159265359f
#define TEST_TWO_PI 6.28318530718f
#define TEST_CONTROL_PERIOD_MS 1U
#define TEST_J4310_POSITION_MAX_RAD 12.5f
#define TEST_J4310_DIRECTION_SIGN (-1.0f)
#define TEST_J4310_TORQUE_MAP_MAX_NM 10.0f
#define TEST_M3508_1_DIRECTION_SIGN 1.0f
#define TEST_M3508_2_DIRECTION_SIGN (-1.0f)
#define TEST_M3508_POSITION_VEL_LIMIT_RAD_S 15.708f
#define TEST_M3508_ACCEL_LIMIT_RAD_S2 62.832f
#define TEST_M2006_POSITION_CUTOFF_RAD 6.45771823238f
#define TEST_GRIPPER_M2006_POSITION_CUTOFF_RAD 12.74090353956f

static const motor_cfg_t upper_motor_cfg[UPPER_MOTOR_COUNT] =
{
    [UPPER_MOTOR_ARM_M3508_1] =
    {
        "arm_m3508_1", MOTOR_MODEL_M3508,
        CAN_BUS_ARM_M3508, NODE_ARM_M3508_1,
        TEST_CONTROL_PERIOD_MS, 0U, true
    },
    [UPPER_MOTOR_ARM_M3508_2] =
    {
        "arm_m3508_2", MOTOR_MODEL_M3508,
        CAN_BUS_ARM_M3508, NODE_ARM_M3508_2,
        TEST_CONTROL_PERIOD_MS, 0U, true
    },
    [UPPER_MOTOR_ARM_J4310] =
    {
        "arm_j4310", MOTOR_MODEL_J4310,
        CAN_BUS_ARM_J4310, NODE_ARM_J4310,
        TEST_CONTROL_PERIOD_MS, 0U, true
    },
    [UPPER_MOTOR_GATE_M2006] =
    {
        "gate_m2006", MOTOR_MODEL_M2006,
        CAN_BUS_AUX, NODE_GATE_M2006,
        TEST_CONTROL_PERIOD_MS, 0U, true
    },
    [UPPER_MOTOR_GRIPPER_M2006] =
    {
        "gripper_m2006", MOTOR_MODEL_M2006,
        CAN_BUS_AUX, NODE_GRIPPER_M2006,
        TEST_CONTROL_PERIOD_MS, 0U, true
    }
};

typedef struct
{
    uint8_t can_bus;
    can_frame_t frame;
} test_tx_t;

static test_tx_t test_tx[TEST_TX_CAPACITY];
static size_t test_tx_count;

/* 功能：模拟 BSP CAN 发送并记录帧；用途：在主机测试中观察电机路由输出；返回 true 表示测试桩接受该帧。 */
bool BspCan_Send(uint8_t can_bus, const can_frame_t *frame)
{
    assert(frame != NULL);
    assert(test_tx_count < TEST_TX_CAPACITY);
    test_tx[test_tx_count].can_bus = can_bus;
    test_tx[test_tx_count].frame = *frame;
    test_tx_count++;
    return true;
}

/* 功能：从测试帧读取大端 16 位有符号数；用途：核对 DJI 电流槽位；返回值表示解码结果。 */
static int16_t Test_ReadI16Be(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

/* 功能：从测试帧读取小端单精度浮点数；用途：核对 J4310 位置速度帧；返回值表示解码结果。 */
static float Test_ReadFloatLe(const uint8_t *data)
{
    uint32_t raw;
    float value;

    raw = (uint32_t)data[0] |
          ((uint32_t)data[1] << 8U) |
          ((uint32_t)data[2] << 16U) |
          ((uint32_t)data[3] << 24U);
    (void)memcpy(&value, &raw, sizeof(value));
    return value;
}

/* 功能：读取 MIT 帧中的 12 位力矩字段；用途：验证软件限幅不会改变固定 TMAX 映射；返回值表示协议原始量。 */
static uint16_t Test_ReadJ4310TorqueRaw(const can_frame_t *frame)
{
    return (uint16_t)((((uint16_t)frame->data[6] & 0x0FU) << 8U) |
                      frame->data[7]);
}

/* 功能：读取 MIT 帧的位置物理量；用途：验证机构正方向已映射为电机负方向；返回值为协议位置目标。 */
static float Test_ReadJ4310MitPosition(const can_frame_t *frame)
{
    uint16_t raw;

    raw = (uint16_t)(((uint16_t)frame->data[0] << 8U) |
                     frame->data[1]);
    return (float)raw * (2.0f * TEST_J4310_POSITION_MAX_RAD) /
           65535.0f - TEST_J4310_POSITION_MAX_RAD;
}

/* 功能：清空测试 CAN 发送记录；用途：隔离各测试场景的输出；无返回值表示桩状态已复位。 */
static void Test_ResetTx(void)
{
    (void)memset(test_tx, 0, sizeof(test_tx));
    test_tx_count = 0U;
}

/* 功能：向电机端口注入一帧模拟 DJI 反馈；用途：建立编码器、速度和电流测试状态；无返回值表示反馈已送入驱动。 */
static void Test_FeedDji(uint8_t can_bus,
                         uint8_t node_id,
                         uint16_t encoder,
                         uint32_t tick_ms)
{
    can_frame_t frame = {0};

    frame.id = 0x200U + node_id;
    frame.dlc = 8U;
    frame.data[0] = (uint8_t)(encoder >> 8U);
    frame.data[1] = (uint8_t)encoder;
    UpperMotorPort_OnFrame(can_bus, &frame, tick_ms);
}

/* 功能：连续注入稳定 DJI 反馈以锁定启动零点；用途：模拟完成零位校准；无返回值表示校准帧已送入。 */
static void Test_LockDjiZero(uint8_t can_bus,
                             uint8_t node_id,
                             uint16_t encoder,
                             uint32_t tick_ms)
{
    uint32_t sample;

    for (sample = 0U; sample < TEST_DJI_ZERO_STABLE_FRAMES; sample++)
    {
        Test_FeedDji(can_bus, node_id, encoder, tick_ms);
    }
}

/* 功能：向指定 DJI 电机注入带速度的反馈；用途：测试速度闭环与控制方向；无返回值表示反馈已送入。 */
static void Test_FeedDjiSpeed(uint8_t can_bus,
                              uint8_t node_id,
                              uint16_t encoder,
                              int16_t rotor_speed_rpm,
                              uint32_t tick_ms)
{
    can_frame_t frame = {0};

    frame.id = 0x200U + node_id;
    frame.dlc = 8U;
    frame.data[0] = (uint8_t)(encoder >> 8U);
    frame.data[1] = (uint8_t)encoder;
    frame.data[2] = (uint8_t)((uint16_t)rotor_speed_rpm >> 8U);
    frame.data[3] = (uint8_t)rotor_speed_rpm;
    UpperMotorPort_OnFrame(can_bus, &frame, tick_ms);
}

/* 功能：判断两个浮点数是否在误差容限内；用途：避免测试受浮点舍入影响；返回 true 表示足够接近。 */
static bool Test_FloatClose(float actual, float expected, float tolerance)
{
    float error = actual - expected;

    return (error >= -tolerance) && (error <= tolerance);
}

/* 功能：验证 DJI 启动反馈能形成正确零点和相对位置；用途：防止上电校准逻辑回归；断言失败会终止测试。 */
static void Test_CheckDjiStartupFeedbackCalibration(void)
{
    m3508_feedback_t m3508_feedback;
    m2006_feedback_t m2006_feedback;

    Test_ResetTx();
    Test_LockDjiZero(2U, 1U, 7000U, 0U);
    Test_LockDjiZero(2U, 2U, 1000U, 0U);
    Test_LockDjiZero(3U, 1U, 8000U, 0U);
    Test_LockDjiZero(3U, 2U, 2000U, 0U);

    assert(M3508_GetFeedback(2U, 1U, &m3508_feedback));
    assert(Test_FloatClose(m3508_feedback.output_pos_rad, 0.0f, 0.000001f));
    assert(M3508_GetFeedback(2U, 2U, &m3508_feedback));
    assert(Test_FloatClose(m3508_feedback.output_pos_rad, 0.0f, 0.000001f));
    assert(M2006_GetFeedback(3U, 1U, &m2006_feedback));
    assert(Test_FloatClose(m2006_feedback.output_pos_rad, 0.0f, 0.000001f));
    assert(M2006_GetFeedback(3U, 2U, &m2006_feedback));
    assert(Test_FloatClose(m2006_feedback.output_pos_rad, 0.0f, 0.000001f));
    Test_FeedDji(2U, 1U, 7100U, 1U);
    Test_FeedDji(2U, 2U, 900U, 1U);
    Test_FeedDji(3U, 1U, 50U, 1U);
    Test_FeedDji(3U, 2U, 1900U, 1U);

    assert(M3508_GetFeedback(2U, 1U, &m3508_feedback));
    assert(m3508_feedback.output_pos_rad > 0.0f);
    assert(M3508_GetFeedback(2U, 2U, &m3508_feedback));
    assert(m3508_feedback.output_pos_rad < 0.0f);
    assert(M2006_GetFeedback(3U, 1U, &m2006_feedback));
    assert(m2006_feedback.output_pos_rad > 0.0f);
    assert(M2006_GetFeedback(3U, 2U, &m2006_feedback));
    assert(m2006_feedback.output_pos_rad < 0.0f);
    assert(test_tx_count == 0U);
}

/* 功能：验证初始化过程不会主动发送电机命令；用途：保证上电阶段只读且安全；断言失败会终止测试。 */
static void Test_CheckStartupIsReadOnly(void)
{
    j4310_online_mit_state_t j4310_online;
    m2006_online_pid_state_t m2006_online;
    m3508_online_pid_state_t m3508_online;

    assert(UpperMotorPort_Init(upper_motor_cfg, UPPER_MOTOR_COUNT));
    assert(J4310_GetOnlineMitState(NODE_ARM_J4310, &j4310_online));
    assert(j4310_online.enabled);
    assert(M3508_GetOnlinePidState(CAN_BUS_ARM_M3508,
                                   NODE_ARM_M3508_1,
                                   &m3508_online));
    assert(!m3508_online.enabled);
    assert(M3508_GetOnlinePidState(CAN_BUS_ARM_M3508,
                                   NODE_ARM_M3508_2,
                                   &m3508_online));
    assert(!m3508_online.enabled);
    assert(M2006_GetOnlinePidState(CAN_BUS_AUX,
                                   NODE_GATE_M2006,
                                   &m2006_online));
    assert(!m2006_online.enabled);
    assert(M2006_GetOnlinePidState(CAN_BUS_AUX,
                                   NODE_GRIPPER_M2006,
                                   &m2006_online));
    assert(!m2006_online.enabled);
    Test_ResetTx();
    UpperMotorPort_BeginCycle(0U);
    assert(UpperMotorPort_Flush());
    assert(test_tx_count == 0U);

    Test_LockDjiZero(2U, 1U, 7000U, 0U);
    Test_LockDjiZero(2U, 2U, 1000U, 0U);
    Test_LockDjiZero(3U, 1U, 8000U, 0U);
    Test_LockDjiZero(3U, 2U, 2000U, 0U);
    UpperMotorPort_BeginCycle(1U);
    assert(UpperMotorPort_Flush());
    assert(test_tx_count == 0U);
}

/* 功能：验证 M2006 分组帧四个电流槽相互独立；用途：防止单节点命令污染同组其他电机；断言失败会终止测试。 */
static void Test_CheckM2006GroupSlotsAreIndependent(void)
{
    motor_cmd_t command = {0};
    m2006_motion_state_t motion;

    assert(UpperMotorPort_Init(upper_motor_cfg, UPPER_MOTOR_COUNT));
    Test_LockDjiZero(3U, NODE_GATE_M2006, 4000U, 10U);
    Test_LockDjiZero(3U, NODE_GRIPPER_M2006, 2000U, 10U);

    Test_ResetTx();
    UpperMotorPort_BeginCycle(10U);
    command.mode = MOTOR_CMD_CURRENT;
    command.current_a = 1.0f;
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_GATE_M2006], &command, NULL));
    command.mode = MOTOR_CMD_STOP;
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_GRIPPER_M2006], &command, NULL));
    assert(UpperMotorPort_Flush());
    assert(test_tx_count == 1U);
    assert(test_tx[0].can_bus == 3U);
    assert(Test_ReadI16Be(&test_tx[0].frame.data[0]) < 0);
    assert(Test_ReadI16Be(&test_tx[0].frame.data[2]) == 0);

    Test_ResetTx();
    UpperMotorPort_BeginCycle(11U);
    command.mode = MOTOR_CMD_STOP;
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_GATE_M2006], &command, NULL));
    command.mode = MOTOR_CMD_CURRENT;
    command.current_a = -1.0f;
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_GRIPPER_M2006], &command, NULL));
    assert(UpperMotorPort_Flush());
    assert(test_tx_count == 1U);
    assert(test_tx[0].can_bus == 3U);
    assert(Test_ReadI16Be(&test_tx[0].frame.data[0]) == 0);
    assert(Test_ReadI16Be(&test_tx[0].frame.data[2]) < 0);

    /* 上一周期处于活动状态的电机不得复用旧命令，除非本周期再次显式调度。 */
    Test_ResetTx();
    UpperMotorPort_BeginCycle(12U);
    command.mode = MOTOR_CMD_CURRENT;
    command.current_a = 1.0f;
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_GATE_M2006], &command, NULL));
    assert(UpperMotorPort_Flush());
    assert(test_tx_count == 1U);
    assert(Test_ReadI16Be(&test_tx[0].frame.data[0]) < 0);
    assert(Test_ReadI16Be(&test_tx[0].frame.data[2]) == 0);

    /* 在启动校准位置下，零度位置目标不会产生电流，也不能触发启动/测试运动。 */
    Test_ResetTx();
    UpperMotorPort_BeginCycle(13U);
    command = (motor_cmd_t){ .mode = MOTOR_CMD_POSITION, .pos_rad = 0.0f };
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_GATE_M2006], &command, NULL));
    assert(UpperMotorPort_Flush());
    assert(test_tx_count == 1U);
    assert(Test_ReadI16Be(&test_tx[0].frame.data[0]) == 0);
    assert(Test_ReadI16Be(&test_tx[0].frame.data[2]) == 0);

    command = (motor_cmd_t){ .mode = MOTOR_CMD_POSITION, .pos_rad = 0.5f };
    UpperMotorPort_BeginCycle(14U);
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_GATE_M2006], &command, NULL));
    assert(M2006_GetMotionState(CAN_BUS_AUX,
                                NODE_GATE_M2006,
                                &motion));
    assert(Test_FloatClose(motion.final_position_rad,
                           -0.5f,
                           0.000001f));
}

/* 功能：执行 CheckGripperUsesIndependentPositionCutoff 场景测试；用途：验证对应输入下的行为和断言；无返回值表示由断言报告结果。 */
static void Test_CheckGripperUsesIndependentPositionCutoff(void)
{
    motor_cmd_t command = {0};
    m2006_feedback_t feedback;
    uint16_t encoder;
    uint32_t sample;

    assert(UpperMotorPort_Init(upper_motor_cfg, UPPER_MOTOR_COUNT));
    encoder = 1000U;
    Test_LockDjiZero(CAN_BUS_AUX, NODE_GATE_M2006, encoder, 10U);
    Test_LockDjiZero(CAN_BUS_AUX, NODE_GRIPPER_M2006, encoder, 10U);
    for (sample = 0U; sample < 80U; sample++)
    {
        encoder = (uint16_t)((encoder + 4000U) % 8192U);
        Test_FeedDji(CAN_BUS_AUX, NODE_GATE_M2006, encoder, 11U + sample);
        Test_FeedDji(CAN_BUS_AUX, NODE_GRIPPER_M2006, encoder, 11U + sample);
    }
    assert(M2006_GetFeedback(CAN_BUS_AUX, NODE_GATE_M2006, &feedback));
    assert(feedback.output_pos_rad > TEST_M2006_POSITION_CUTOFF_RAD);
    assert(feedback.output_pos_rad <
           TEST_GRIPPER_M2006_POSITION_CUTOFF_RAD);

    Test_ResetTx();
    UpperMotorPort_BeginCycle(33U);
    command.mode = MOTOR_CMD_CURRENT;
    command.current_a = 1.0f;
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_GATE_M2006], &command, NULL));
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_GRIPPER_M2006], &command, NULL));
    assert(UpperMotorPort_Flush());
    assert(test_tx_count == 1U);
    assert(test_tx[0].can_bus == CAN_BUS_AUX);
    assert(Test_ReadI16Be(&test_tx[0].frame.data[0]) == 0);
    assert(Test_ReadI16Be(&test_tx[0].frame.data[2]) > 0);
}

/* 功能：验证 M3508 分组帧四个电流槽相互独立；用途：防止单节点命令污染同组其他电机；断言失败会终止测试。 */
static void Test_CheckM3508GroupSlotsAreIndependent(void)
{
    motor_cmd_t command = {0};
    m3508_motion_state_t motion;

    assert(UpperMotorPort_Init(upper_motor_cfg, UPPER_MOTOR_COUNT));
    Test_LockDjiZero(2U, 1U, 4000U, 10U);
    Test_LockDjiZero(2U, 2U, 2000U, 10U);

    Test_ResetTx();
    UpperMotorPort_BeginCycle(10U);
    command.mode = MOTOR_CMD_CURRENT;
    command.current_a = 1.0f;
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_M3508_1], &command, NULL));
    command.mode = MOTOR_CMD_STOP;
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_M3508_2], &command, NULL));
    assert(UpperMotorPort_Flush());
    assert(test_tx_count == 1U);
    assert(test_tx[0].can_bus == 2U);
    assert(Test_ReadI16Be(&test_tx[0].frame.data[0]) > 0);
    assert(Test_ReadI16Be(&test_tx[0].frame.data[2]) == 0);

    Test_ResetTx();
    UpperMotorPort_BeginCycle(11U);
    command.mode = MOTOR_CMD_STOP;
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_M3508_1], &command, NULL));
    command.mode = MOTOR_CMD_CURRENT;
    command.current_a = -1.0f;
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_M3508_2], &command, NULL));
    assert(UpperMotorPort_Flush());
    assert(test_tx_count == 1U);
    assert(test_tx[0].can_bus == 2U);
    assert(Test_ReadI16Be(&test_tx[0].frame.data[0]) == 0);
    assert(Test_ReadI16Be(&test_tx[0].frame.data[2]) > 0);

    command = (motor_cmd_t){ .mode = MOTOR_CMD_POSITION, .pos_rad = 0.5f };
    UpperMotorPort_BeginCycle(12U);
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_M3508_1], &command, NULL));
    assert(M3508_GetMotionState(CAN_BUS_ARM_M3508,
                                NODE_ARM_M3508_1,
                                &motion));
    assert(Test_FloatClose(motion.final_position_rad,
                           0.5f,
                           0.000001f));
}

/* 功能：验证 DJI 反馈中断后重新稳定可恢复零点校准；用途：覆盖掉线重连路径；断言失败会终止测试。 */
static void Test_CheckDjiRestartFeedbackCalibration(void)
{
    upper_dji_diagnostic_t diagnostics[4];
    size_t diagnostic_count;
    m3508_feedback_t m3508_feedback;
    m2006_feedback_t m2006_feedback;

    assert(UpperMotorPort_Init(upper_motor_cfg, UPPER_MOTOR_COUNT));
    Test_ResetTx();
    /* 拒绝短暂的首帧零值，然后将稳定反馈锁定为本次 H723 启动的输出轴相对零点。 */
    Test_FeedDji(2U, 1U, 0U, 0U);
    Test_FeedDji(3U, 1U, 0U, 0U);
    assert(!M3508_GetFeedback(2U, 1U, &m3508_feedback));
    assert(!M2006_GetFeedback(3U, 1U, &m2006_feedback));
    Test_LockDjiZero(2U, 1U, 6000U, 0U);
    Test_LockDjiZero(3U, 1U, 5000U, 0U);
    assert(M3508_GetFeedback(2U, 1U, &m3508_feedback));
    assert(Test_FloatClose(m3508_feedback.output_pos_rad, 0.0f, 0.000001f));
    assert(M2006_GetFeedback(3U, 1U, &m2006_feedback));
    assert(Test_FloatClose(m2006_feedback.output_pos_rad, 0.0f, 0.000001f));
    diagnostic_count = UpperMotorPort_GetDjiDiagnostics(
                           0U, diagnostics, 4U);
    assert(diagnostic_count == 4U);
    assert(diagnostics[0].model == MOTOR_MODEL_M3508);
    assert(diagnostics[0].can_bus == 2U);
    assert(diagnostics[0].node_id == 1U);
    assert(diagnostics[0].feedback_received);
    assert(diagnostics[0].zero_valid);
    assert(diagnostics[0].feedback_fresh);
    assert(Test_FloatClose(diagnostics[0].rotor_position_rad,
                           6000.0f * 6.28318530718f / 8192.0f,
                           0.000001f));
    assert(Test_FloatClose(diagnostics[0].zero_rotor_position_rad,
                           diagnostics[0].rotor_position_rad,
                           0.000001f));
    assert(Test_FloatClose(diagnostics[0].relative_output_position_rad,
                           0.0f,
                           0.000001f));
    assert(diagnostics[2].model == MOTOR_MODEL_M2006);
    assert(diagnostics[2].can_bus == 3U);
    assert(diagnostics[2].node_id == 1U);
    assert(diagnostics[2].zero_valid);
    assert(Test_FloatClose(diagnostics[2].zero_rotor_position_rad,
                           5000.0f * 6.28318530718f / 8192.0f,
                           0.000001f));

    Test_FeedDji(2U, 1U, 6100U, 2U);
    Test_FeedDji(3U, 1U, 5100U, 2U);
    assert(M3508_GetFeedback(2U, 1U, &m3508_feedback));
    assert(m3508_feedback.output_pos_rad > 0.0f);
    assert(M2006_GetFeedback(3U, 1U, &m2006_feedback));
    assert(m2006_feedback.output_pos_rad > 0.0f);
    diagnostic_count = UpperMotorPort_GetDjiDiagnostics(
                           2U, diagnostics, 4U);
    assert(diagnostic_count == 4U);
    assert(diagnostics[0].relative_output_position_rad > 0.0f);
    assert(diagnostics[2].relative_output_position_rad < 0.0f);
    assert(test_tx_count == 0U);
}

/* 功能：注入一帧正常 J4310 反馈；用途：为模式和健康测试建立在线状态；无返回值表示反馈已送入端口。 */
static void Test_FeedJ4310(uint32_t tick_ms)
{
    can_frame_t frame = {0};

    frame.dlc = 8U;
    frame.id = CAN_J4310_MASTER_ID;
    frame.data[0] = NODE_ARM_J4310;
    UpperMotorPort_OnFrame(1U, &frame, tick_ms);
}

/* 功能：注入带指定状态码的 J4310 反馈；用途：测试已使能状态与协议故障；无返回值表示反馈已送入。 */
static void Test_FeedJ4310State(uint32_t tick_ms, uint8_t state)
{
    can_frame_t frame = {0};

    frame.id = CAN_J4310_MASTER_ID;
    frame.dlc = 8U;
    frame.data[0] = (uint8_t)((state << 4U) | NODE_ARM_J4310);
    UpperMotorPort_OnFrame(1U, &frame, tick_ms);
}

/* 功能：向被测端口注入一帧 J4310 位置反馈；用途：建立或更新连续角度跟踪状态；无返回值表示反馈已送入。 */
static void Test_FeedJ4310Position(float mechanism_position_rad,
                                   uint32_t tick_ms,
                                   uint8_t state)
{
    can_frame_t frame = {0};
    float protocol_position_rad;
    float normalized;
    uint16_t raw;

    protocol_position_rad = mechanism_position_rad *
                            TEST_J4310_DIRECTION_SIGN;
    normalized = (protocol_position_rad + TEST_J4310_POSITION_MAX_RAD) /
                 (2.0f * TEST_J4310_POSITION_MAX_RAD);
    raw = (uint16_t)(normalized * 65535.0f + 0.5f);
    frame.id = CAN_J4310_MASTER_ID;
    frame.dlc = 8U;
    frame.data[0] = (uint8_t)((state << 4U) | NODE_ARM_J4310);
    frame.data[1] = (uint8_t)(raw >> 8U);
    frame.data[2] = (uint8_t)raw;
    UpperMotorPort_OnFrame(CAN_BUS_ARM_J4310, &frame, tick_ms);
}

/* 功能：执行 CheckJ4310PowerCyclePositionUnwrap 场景测试；用途：验证对应输入下的行为和断言；无返回值表示由断言报告结果。 */
static void Test_CheckJ4310PowerCyclePositionUnwrap(void)
{
    motor_cmd_t command = {0};
    upper_j4310_feedback_t feedback;

    assert(UpperMotorPort_Init(upper_motor_cfg, UPPER_MOTOR_COUNT));
    Test_FeedJ4310Position(0.5f * TEST_PI, 10U, 0x01U);
    UpperMotorPort_BeginCycle(10U);
    assert(UpperMotorPort_GetJ4310Feedback(CAN_BUS_ARM_J4310,
                                           NODE_ARM_J4310,
                                           &feedback));
    assert(Test_FloatClose(feedback.position_rad,
                           0.5f * TEST_PI,
                           0.001f));

    /* 电机重启时 MCU 保持供电。单圈绝对编码器会将物理 200 度报告为等效的 -160 度。 */
    Test_FeedJ4310Position(-160.0f * TEST_PI / 180.0f,
                           100U,
                           0x01U);
    UpperMotorPort_BeginCycle(100U);
    assert(UpperMotorPort_GetJ4310Feedback(CAN_BUS_ARM_J4310,
                                           NODE_ARM_J4310,
                                           &feedback));
    assert(Test_FloatClose(feedback.position_rad,
                           200.0f * TEST_PI / 180.0f,
                           0.001f));
    assert(UpperMotorPort_EnableJ4310(CAN_BUS_ARM_J4310,
                                      NODE_ARM_J4310));

    command.mode = MOTOR_CMD_MIT;
    command.kp = 5.0f;
    command.kd = 0.5f;
    command.pos_rad = 0.0f;
    Test_ResetTx();
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_J4310], &command, NULL));
    assert(test_tx_count == 1U);
    assert(Test_FloatClose(Test_ReadJ4310MitPosition(&test_tx[0].frame),
                           TEST_TWO_PI,
                           0.001f));

    /* 沿命令分支运动到物理零点。逻辑反馈必须从 200 度收敛到 0 度，且不能改变圈数。 */
    Test_FeedJ4310Position(-200.0f * TEST_PI / 180.0f, 110U, 0x01U);
    Test_FeedJ4310Position(-250.0f * TEST_PI / 180.0f, 120U, 0x01U);
    Test_FeedJ4310Position(-300.0f * TEST_PI / 180.0f, 130U, 0x01U);
    Test_FeedJ4310Position(-350.0f * TEST_PI / 180.0f, 140U, 0x01U);
    Test_FeedJ4310Position(-TEST_TWO_PI, 150U, 0x01U);
    UpperMotorPort_BeginCycle(150U);
    assert(UpperMotorPort_GetJ4310Feedback(CAN_BUS_ARM_J4310,
                                           NODE_ARM_J4310,
                                           &feedback));
    assert(Test_FloatClose(feedback.position_rad, 0.0f, 0.002f));

    command.pos_rad = 0.5f * TEST_PI;
    Test_ResetTx();
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_J4310], &command, NULL));
    assert(test_tx_count == 1U);
    assert(Test_FloatClose(Test_ReadJ4310MitPosition(&test_tx[0].frame),
                           1.5f * TEST_PI,
                           0.002f));
}

/* 功能：验证软件力矩限幅与协议 TMAX 完全分离；用途：防止 GUI 限幅改变 MIT 编解码比例；断言失败会终止测试。 */
static void Test_CheckJ4310TorqueLimitMapping(void)
{
    can_frame_t command = {0};
    can_frame_t feedback_frame = {0};
    j4310_feedback_t feedback;

    assert(J4310_SetTorqueLimit(NODE_ARM_J4310, 2.0f));
    assert(!J4310_SetTorqueLimit(NODE_ARM_J4310, 10.1f));
    assert(J4310_BuildMit(NODE_ARM_J4310,
                          0.0f,
                          0.0f,
                          0.0f,
                          0.0f,
                          8.0f,
                          &command));
    assert(Test_ReadJ4310TorqueRaw(&command) == 2457U);

    feedback_frame.id = CAN_J4310_MASTER_ID;
    feedback_frame.dlc = 8U;
    feedback_frame.data[0] = NODE_ARM_J4310;
    feedback_frame.data[4] = 0x0BU;
    feedback_frame.data[5] = 0xFFU;
    assert(J4310_OnFrame(&feedback_frame, 10U));
    assert(J4310_GetFeedback(NODE_ARM_J4310, &feedback));
    assert(Test_FloatClose(feedback.torque_nm, 5.0f, 0.01f));
    assert(J4310_SetTorqueLimit(NODE_ARM_J4310,
                                TEST_J4310_TORQUE_MAP_MAX_NM));
}

/* 功能：验证达妙接收拒绝原因计数；用途：让现场能区分帧格式、Master ID 与反馈 ID 错误；断言失败会终止测试。 */
static void Test_CheckJ4310RxDiagnostics(void)
{
    can_frame_t frame = {0};
    upper_j4310_rx_diagnostic_t before;
    upper_j4310_rx_diagnostic_t diagnostic;

    assert(UpperMotorPort_GetJ4310RxDiagnostic(
        CAN_BUS_ARM_J4310, NODE_ARM_J4310, &before));
    frame.id = CAN_J4310_MASTER_ID;
    frame.dlc = 7U;
    UpperMotorPort_OnFrame(CAN_BUS_ARM_J4310, &frame, 20U);
    frame.id = 0x015U;
    frame.dlc = 8U;
    frame.data[0] = NODE_ARM_J4310;
    UpperMotorPort_OnFrame(CAN_BUS_ARM_J4310, &frame, 21U);
    frame.id = CAN_J4310_MASTER_ID;
    frame.data[0] = 0x05U;
    UpperMotorPort_OnFrame(CAN_BUS_ARM_J4310, &frame, 22U);
    frame.data[0] = NODE_ARM_J4310;
    UpperMotorPort_OnFrame(CAN_BUS_ARM_J4310, &frame, 23U);

    assert(UpperMotorPort_GetJ4310RxDiagnostic(
        CAN_BUS_ARM_J4310, NODE_ARM_J4310, &diagnostic));
    assert(diagnostic.frames_seen == before.frames_seen + 4U);
    assert(diagnostic.accepted_frames == before.accepted_frames + 1U);
    assert(diagnostic.rejected_format_frames ==
           before.rejected_format_frames + 1U);
    assert(diagnostic.rejected_master_id_frames ==
           before.rejected_master_id_frames + 1U);
    assert(diagnostic.rejected_feedback_id_frames ==
           before.rejected_feedback_id_frames + 1U);
    assert(diagnostic.last_can_id == CAN_J4310_MASTER_ID);
    assert(diagnostic.last_dlc == 8U);
    assert(diagnostic.last_data0 == NODE_ARM_J4310);
    assert(diagnostic.last_result == J4310_RX_ACCEPTED);
}

/* 功能：判断测试帧是否为指定 J4310 特殊命令；用途：识别使能、失能和清错帧；返回 true 表示格式与命令匹配。 */
static bool Test_IsJ4310Special(const can_frame_t *frame, uint8_t command)
{
    size_t index;

    if ((frame == NULL) || (frame->dlc != 8U) ||
        (frame->data[7] != command))
    {
        return false;
    }
    for (index = 0U; index < 7U; index++)
    {
        if (frame->data[index] != 0xFFU)
        {
            return false;
        }
    }
    return true;
}

/* 功能：在指定时刻对全部配置电机执行一次发送调度；用途：构造完整路由周期；无返回值表示分组帧也已刷新。 */
static void Test_SendAll(uint32_t tick_ms)
{
    static const float current_a[UPPER_MOTOR_COUNT] =
    {
        1.0f,
        2.0f,
        0.0f,
        1.0f,
        -1.0f
    };
    size_t index;

    UpperMotorPort_BeginCycle(tick_ms);
    for (index = 0U; index < UPPER_MOTOR_COUNT; index++)
    {
        motor_cmd_t cmd = {0};

        if (upper_motor_cfg[index].model == MOTOR_MODEL_J4310)
        {
            cmd.mode = MOTOR_CMD_MIT;
            cmd.kp = 20.0f;
            cmd.kd = 0.5f;
        }
        else
        {
            cmd.mode = MOTOR_CMD_CURRENT;
            cmd.current_a = current_a[index];
        }
        assert(UpperMotorPort_Send(&upper_motor_cfg[index], &cmd, NULL));
    }
    assert(UpperMotorPort_Flush());
}

/* 功能：验证电机拓扑的总线、节点和型号映射；用途：防止配置表与协议路由不一致；断言失败会终止测试。 */
static void Test_CheckTopology(void)
{
    assert(TEST_CONTROL_PERIOD_MS == 1U);
    assert(TEST_M3508_1_DIRECTION_SIGN == 1.0f);
    assert(TEST_M3508_2_DIRECTION_SIGN == -1.0f);
    assert(upper_motor_cfg[UPPER_MOTOR_ARM_M3508_1].can_bus == 2U);
    assert(upper_motor_cfg[UPPER_MOTOR_ARM_M3508_1].node_id == 1U);
    assert(upper_motor_cfg[UPPER_MOTOR_ARM_M3508_2].can_bus == 2U);
    assert(upper_motor_cfg[UPPER_MOTOR_ARM_M3508_2].node_id == 2U);
    assert(upper_motor_cfg[UPPER_MOTOR_ARM_J4310].can_bus == 1U);
    assert(upper_motor_cfg[UPPER_MOTOR_ARM_J4310].node_id == 0x06U);
    assert(upper_motor_cfg[UPPER_MOTOR_GATE_M2006].can_bus == 3U);
    assert(upper_motor_cfg[UPPER_MOTOR_GATE_M2006].node_id ==
           NODE_GATE_M2006);
    assert(NODE_GATE_M2006 == 1U);
    assert(upper_motor_cfg[UPPER_MOTOR_GRIPPER_M2006].can_bus == 3U);
    assert(upper_motor_cfg[UPPER_MOTOR_GRIPPER_M2006].node_id ==
           NODE_GRIPPER_M2006);
    assert(NODE_GRIPPER_M2006 == 2U);
}

/* 功能：验证首个控制周期的使能与 CAN 帧输出；用途：覆盖上电后的首次发送行为；断言失败会终止测试。 */
static void Test_CheckFirstCycle(void)
{
    assert(test_tx_count == 3U);
    assert(test_tx[0].can_bus == 1U);
    assert(test_tx[0].frame.id == 0x06U);
    assert(test_tx[0].frame.data[7] == 0xFCU);

    assert(test_tx[1].can_bus == 2U);
    assert(test_tx[1].frame.id == 0x200U);
    assert(Test_ReadI16Be(&test_tx[1].frame.data[0]) > 0);
    assert(Test_ReadI16Be(&test_tx[1].frame.data[2]) < 0);
    assert(Test_ReadI16Be(&test_tx[1].frame.data[4]) == 0);
    assert(Test_ReadI16Be(&test_tx[1].frame.data[6]) == 0);

    assert(test_tx[2].can_bus == 3U);
    assert(test_tx[2].frame.id == 0x200U);
    assert(Test_ReadI16Be(&test_tx[2].frame.data[0]) < 0);
    assert(Test_ReadI16Be(&test_tx[2].frame.data[2]) < 0);
    assert(Test_ReadI16Be(&test_tx[2].frame.data[4]) == 0);
    assert(Test_ReadI16Be(&test_tx[2].frame.data[6]) == 0);
}

/* 功能：验证 J4310 必须由反馈确认使能；用途：防止单次 0xFC 未被电机接受后永久改发 MIT；断言失败会终止测试。 */
static void Test_CheckJ4310EnableConfirmation(void)
{
    motor_cmd_t command = {0};
    upper_j4310_tx_diagnostic_t diagnostic;

    command.mode = MOTOR_CMD_MIT;
    command.kp = 20.0f;
    command.kd = 0.5f;
    Test_FeedJ4310State(10U, 0x00U);

    Test_ResetTx();
    UpperMotorPort_BeginCycle(10U);
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_J4310], &command, NULL));
    assert(test_tx_count == 1U);
    assert(Test_IsJ4310Special(&test_tx[0].frame, 0xFCU));

    Test_ResetTx();
    UpperMotorPort_BeginCycle(11U);
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_J4310], &command, NULL));
    assert(test_tx_count == 0U);

    Test_ResetTx();
    UpperMotorPort_BeginCycle(30U);
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_J4310], &command, NULL));
    assert(test_tx_count == 1U);
    assert(Test_IsJ4310Special(&test_tx[0].frame, 0xFCU));

    Test_FeedJ4310State(30U, 0x01U);
    Test_ResetTx();
    UpperMotorPort_BeginCycle(31U);
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_J4310], &command, NULL));
    assert(test_tx_count == 1U);
    assert(!Test_IsJ4310Special(&test_tx[0].frame, 0xFCU));
    assert(UpperMotorPort_GetJ4310TxDiagnostic(
        CAN_BUS_ARM_J4310, NODE_ARM_J4310, &diagnostic));
    assert(diagnostic.attempted_frames == 3U);
    assert(diagnostic.queued_frames == 3U);
    assert(diagnostic.failed_frames == 0U);
    assert(diagnostic.enable_frames == 2U);
    assert(diagnostic.mit_frames == 1U);
    assert(diagnostic.disable_frames == 0U);
    assert(diagnostic.last_can_id == 0x006U);
    assert(diagnostic.last_dlc == 8U);
    assert(diagnostic.enable_confirmed);
    assert(diagnostic.feedback_state == 0x01U);

    Test_FeedJ4310State(32U, 0x02U);
    Test_ResetTx();
    UpperMotorPort_BeginCycle(32U);
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_J4310], &command, NULL));
    assert(test_tx_count == 1U);
    assert(Test_IsJ4310Special(&test_tx[0].frame, 0xFDU));
}

/* 功能：执行 CheckJ4310EnableOnly 场景测试；用途：验证对应输入下的行为和断言；无返回值表示由断言报告结果。 */
static void Test_CheckJ4310EnableOnly(void)
{
    assert(UpperMotorPort_Init(upper_motor_cfg, UPPER_MOTOR_COUNT));
    Test_ResetTx();
    UpperMotorPort_BeginCycle(50U);
    assert(UpperMotorPort_EnableJ4310(CAN_BUS_ARM_J4310,
                                      NODE_ARM_J4310));
    assert(test_tx_count == 1U);
    assert(Test_IsJ4310Special(&test_tx[0].frame, 0xFCU));

    Test_FeedJ4310State(50U, 0x01U);
    Test_ResetTx();
    UpperMotorPort_BeginCycle(51U);
    assert(UpperMotorPort_EnableJ4310(CAN_BUS_ARM_J4310,
                                      NODE_ARM_J4310));
    assert(test_tx_count == 0U);
}

/* 功能：验证 J4310 各参考控制模式的帧格式和状态切换；用途：覆盖 MIT、位置速度及速度路由；断言失败会终止测试。 */
static void Test_CheckJ4310ReferenceModes(motor_cmd_t *command,
                                           uint32_t *tick_ms)
{
    command->mode = MOTOR_CMD_POSITION_VELOCITY;
    command->pos_rad = 1.0f;
    command->vel_rad_s = 2.0f;
    Test_ResetTx();
    UpperMotorPort_BeginCycle((*tick_ms)++);
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_J4310], command, NULL));
    assert(test_tx_count == 1U);
    assert(test_tx[0].frame.id == 0x006U);
    assert(Test_IsJ4310Special(&test_tx[0].frame, 0xFDU));

    Test_ResetTx();
    UpperMotorPort_BeginCycle((*tick_ms)++);
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_J4310], command, NULL));
    assert(test_tx_count == 1U);
    assert(test_tx[0].frame.id == 0x106U);
    assert(Test_IsJ4310Special(&test_tx[0].frame, 0xFCU));
    Test_FeedJ4310State((*tick_ms) - 1U, 0x01U);

    Test_ResetTx();
    UpperMotorPort_BeginCycle((*tick_ms)++);
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_J4310], command, NULL));
    assert(test_tx_count == 1U);
    assert(test_tx[0].frame.id == 0x106U);
    assert(test_tx[0].frame.dlc == 8U);
    assert(Test_FloatClose(Test_ReadFloatLe(&test_tx[0].frame.data[0]),
                           -1.0f,
                           0.000001f));
    assert(Test_FloatClose(Test_ReadFloatLe(&test_tx[0].frame.data[4]),
                           -2.0f,
                           0.000001f));

    command->mode = MOTOR_CMD_VELOCITY;
    command->vel_rad_s = -1.0f;
    Test_ResetTx();
    UpperMotorPort_BeginCycle((*tick_ms)++);
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_J4310], command, NULL));
    assert(test_tx_count == 1U);
    assert(test_tx[0].frame.id == 0x106U);
    assert(Test_IsJ4310Special(&test_tx[0].frame, 0xFDU));

    Test_ResetTx();
    UpperMotorPort_BeginCycle((*tick_ms)++);
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_J4310], command, NULL));
    assert(test_tx_count == 1U);
    assert(test_tx[0].frame.id == 0x206U);
    assert(Test_IsJ4310Special(&test_tx[0].frame, 0xFCU));
    Test_FeedJ4310State((*tick_ms) - 1U, 0x01U);

    Test_ResetTx();
    UpperMotorPort_BeginCycle((*tick_ms)++);
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_J4310], command, NULL));
    assert(test_tx_count == 1U);
    assert(test_tx[0].frame.id == 0x206U);
    assert(test_tx[0].frame.dlc == 4U);
    assert(Test_FloatClose(Test_ReadFloatLe(test_tx[0].frame.data),
                           1.0f,
                           0.000001f));

    command->mode = MOTOR_CMD_MIT;
    command->kp = 20.0f;
    command->kd = 0.5f;
    Test_ResetTx();
    UpperMotorPort_BeginCycle((*tick_ms)++);
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_J4310], command, NULL));
    assert(test_tx_count == 1U);
    assert(test_tx[0].frame.id == 0x206U);
    assert(Test_IsJ4310Special(&test_tx[0].frame, 0xFDU));

    Test_ResetTx();
    UpperMotorPort_BeginCycle((*tick_ms)++);
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_J4310], command, NULL));
    assert(test_tx_count == 1U);
    assert(test_tx[0].frame.id == 0x006U);
    assert(Test_IsJ4310Special(&test_tx[0].frame, 0xFCU));
}

/* 功能：验证 DJI 电机速度与位置参考控制输出；用途：覆盖双环 PID、零点和分组发送；断言失败会终止测试。 */
static void Test_CheckDjiReferenceControl(void)
{
    m3508_feedback_t m3508_feedback;
    m3508_motion_state_t m3508_motion;
    m3508_autotune_state_t m3508_tune;
    m3508_pid_cfg_t m3508_speed_pid;
    m2006_feedback_t m2006_feedback;
    m2006_motion_state_t m2006_motion;
    m2006_autotune_state_t m2006_tune;
    can_frame_t frame = {0};
    float m3508_target;
    float m2006_target;
    float max_velocity;
    float max_acceleration;
    int16_t current_raw;
    uint32_t tick_ms;

    assert(UpperMotorPort_Init(upper_motor_cfg, UPPER_MOTOR_COUNT));
    frame.dlc = 8U;
    frame.id = 0x201U;
    frame.data[0] = 0x10U;
    frame.data[1] = 0x00U;
    Test_LockDjiZero(2U, 1U, 0x1000U, 1000U);
    Test_LockDjiZero(3U, 1U, 0x1000U, 1000U);
    assert(M3508_GetFeedback(2U, 1U, &m3508_feedback));
    assert(M2006_GetFeedback(3U, 1U, &m2006_feedback));
    m3508_target = m3508_feedback.output_pos_rad + 1.0f;
    m2006_target = m2006_feedback.output_pos_rad - 0.5f;
    max_velocity = 0.0f;
    max_acceleration = 0.0f;

    for (tick_ms = 1001U; tick_ms <= 1500U; tick_ms++)
    {
        UpperMotorPort_OnFrame(2U, &frame, tick_ms);
        UpperMotorPort_OnFrame(3U, &frame, tick_ms);
        assert(M3508_SetTarget(2U, 1U, M3508_MODE_POSITION,
                               m3508_target, tick_ms));
        assert(M2006_SetTarget(3U, 1U, M2006_MODE_POSITION,
                               m2006_target, tick_ms));
        assert(M3508_CalcCurrentRaw(2U, 1U, tick_ms, &current_raw));
        assert(M2006_CalcCurrentRaw(3U, 1U, tick_ms, &current_raw));
        assert(M3508_GetMotionState(2U, 1U, &m3508_motion));
        assert(M2006_GetMotionState(3U, 1U, &m2006_motion));
        if (tick_ms == 1250U)
        {
            assert(!m2006_motion.trajectory_active);
        }
        if (tick_ms == 1350U)
        {
            assert(!m3508_motion.trajectory_active);
        }
        if (m3508_motion.reference_velocity_rad_s > max_velocity)
        {
            max_velocity = m3508_motion.reference_velocity_rad_s;
        }
        if (m3508_motion.reference_acceleration_rad_s2 > max_acceleration)
        {
            max_acceleration = m3508_motion.reference_acceleration_rad_s2;
        }
    }
    assert(!m3508_motion.trajectory_active);
    assert(!m2006_motion.trajectory_active);
    assert(Test_FloatClose(m3508_motion.reference_position_rad,
                           m3508_target,
                           0.00001f));
    assert(Test_FloatClose(m2006_motion.reference_position_rad,
                           m2006_target,
                           0.00001f));
    assert(max_velocity <= TEST_M3508_POSITION_VEL_LIMIT_RAD_S + 0.001f);
    assert(max_acceleration <= TEST_M3508_ACCEL_LIMIT_RAD_S2 + 0.01f);

    assert(M3508_CalcCurrentRaw(2U, 1U, 1701U, &current_raw));
    assert(current_raw != 0);
    assert(M3508_GetMotionState(2U, 1U, &m3508_motion));
    assert(!Test_FloatClose(m3508_motion.current_command_a,
                            0.0f,
                            0.000001f));
    Test_FeedDjiSpeed(2U, 1U, 0x1000U, 0, 1702U);
    assert(M3508_SetTarget(2U, 1U, M3508_MODE_POSITION,
                           m3508_target, 1702U));
    assert(M3508_CalcCurrentRaw(2U, 1U, 1702U, &current_raw));
    assert(M3508_GetMotionState(2U, 1U, &m3508_motion));
    assert(!m3508_motion.trajectory_active);
    assert(Test_FloatClose(m3508_motion.trajectory_progress,
                           1.0f,
                           0.000001f));

    assert(M3508_StartSpeedAutoTune(2U, 1U, 1.0f, 0.5f, 5.0f,
                                    1702U));
    assert(M3508_GetAutoTuneState(2U, 1U, &m3508_tune));
    assert(m3508_tune.status == M3508_AUTOTUNE_RUNNING);
    assert(M3508_CalcCurrentRaw(2U, 1U, 1702U, &current_raw));
    assert(current_raw > 0);
    for (tick_ms = 1800U; tick_ms <= 2800U; tick_ms += 200U)
    {
        Test_FeedDjiSpeed(2U, 1U, 0x1000U, 0, tick_ms - 50U);
        Test_FeedDjiSpeed(2U, 1U, 0x1000U, 183, tick_ms);
        assert(M3508_CalcCurrentRaw(2U, 1U, tick_ms, &current_raw));
        if (tick_ms < 2800U)
        {
            assert(current_raw < 0);
            Test_FeedDjiSpeed(2U, 1U, 0x1000U, 0, tick_ms + 50U);
            Test_FeedDjiSpeed(2U, 1U, 0x1000U, -183, tick_ms + 100U);
            assert(M3508_CalcCurrentRaw(2U, 1U, tick_ms + 100U,
                                        &current_raw));
            assert(current_raw > 0);
        }
    }
    assert(M3508_GetAutoTuneState(2U, 1U, &m3508_tune));
    assert(m3508_tune.status == M3508_AUTOTUNE_COMPLETE);
    assert(m3508_tune.ultimate_period_s > 0.0f);
    assert(m3508_tune.ultimate_gain > 0.0f);
    assert(m3508_tune.result.kp > 0.0f);
    assert(m3508_tune.result.ki > 0.0f);
    assert(M3508_GetSpeedPid(2U, 1U, &m3508_speed_pid));
    assert(Test_FloatClose(m3508_speed_pid.kp,
                           m3508_tune.result.kp,
                           0.000001f));
    assert(Test_FloatClose(m3508_speed_pid.ki,
                           m3508_tune.result.ki,
                           0.000001f));

    Test_FeedDjiSpeed(3U, 1U, 0x1000U, 0, 2800U);
    assert(M2006_StartSpeedAutoTune(3U, 1U, 1.0f, 0.5f, 5.0f,
                                    2800U));
    assert(M2006_GetAutoTuneState(3U, 1U, &m2006_tune));
    assert(m2006_tune.status == M2006_AUTOTUNE_RUNNING);
    assert(M2006_CancelAutoTune(3U, 1U));
}

/* 功能：运行上层电机路由的全部主机断言；用途：检查拓扑、反馈校准、协议帧和控制模式；返回 0 表示全部通过。 */
int main(void)
{
    motor_cmd_t j4310_cmd = {0};
    upper_motor_fault_t fault;
    upper_motor_health_t health;
    upper_j4310_feedback_t j4310_feedback;
    float j4310_position_rad;
    uint32_t j4310_mode_tick_ms;

    Test_CheckTopology();
    Test_CheckJ4310PowerCyclePositionUnwrap();
    assert(UpperMotorPort_Init(upper_motor_cfg, UPPER_MOTOR_COUNT));
    Test_CheckJ4310TorqueLimitMapping();
    Test_CheckJ4310RxDiagnostics();
    Test_CheckJ4310EnableOnly();
    assert(UpperMotorPort_Init(upper_motor_cfg, UPPER_MOTOR_COUNT));
    Test_CheckJ4310EnableConfirmation();
    assert(UpperMotorPort_Init(upper_motor_cfg, UPPER_MOTOR_COUNT));
    Test_CheckStartupIsReadOnly();
    Test_CheckM3508GroupSlotsAreIndependent();
    Test_CheckM2006GroupSlotsAreIndependent();
    Test_CheckGripperUsesIndependentPositionCutoff();
    assert(UpperMotorPort_Init(upper_motor_cfg, UPPER_MOTOR_COUNT));
    Test_CheckDjiStartupFeedbackCalibration();

    Test_FeedJ4310State(0U, 0x0BU);
    assert(UpperMotorPort_GetHealth(0U, &health));
    assert(UpperMotorPort_GetPendingFault(&fault));
    assert(fault.model == MOTOR_MODEL_J4310);
    assert(fault.can_bus == 1U);
    assert(fault.node_id == 0x06U);
    assert(fault.error_code == 0x0BU);
    UpperMotorPort_MarkFaultSent(fault.sequence);
    Test_FeedJ4310(0U);
    assert(UpperMotorPort_GetHealth(0U, &health));

    Test_ResetTx();
    Test_SendAll(1U);
    Test_CheckFirstCycle();

    Test_FeedJ4310State(1U, 0x01U);
    Test_ResetTx();
    Test_SendAll(2U);
    assert(test_tx_count == 3U);
    assert(test_tx[0].can_bus == 1U);
    assert(test_tx[0].frame.id == 0x06U);
    assert(test_tx[0].frame.data[7] != 0xFCU);
    assert(test_tx[1].can_bus == 2U);
    assert(test_tx[1].frame.id == 0x200U);

    j4310_cmd.mode = MOTOR_CMD_STOP;
    Test_ResetTx();
    UpperMotorPort_BeginCycle(3U);
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_J4310], &j4310_cmd, NULL));
    assert(test_tx_count == 1U);
    assert(Test_IsJ4310Special(&test_tx[0].frame, 0xFDU));

    j4310_cmd.mode = MOTOR_CMD_MIT;
    j4310_cmd.kp = 20.0f;
    j4310_cmd.kd = 0.5f;
    j4310_cmd.pos_rad = 1.0f;
    Test_ResetTx();
    UpperMotorPort_BeginCycle(100U);
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_J4310], &j4310_cmd, NULL));
    assert(test_tx_count == 1U);
    assert(Test_IsJ4310Special(&test_tx[0].frame, 0xFCU));

    Test_FeedJ4310State(100U, 0x01U);
    assert(UpperMotorPort_GetHealth(100U, &health));
    assert(!UpperMotorPort_GetPendingFault(&fault));
    Test_ResetTx();
    UpperMotorPort_BeginCycle(101U);
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_J4310], &j4310_cmd, NULL));
    assert(test_tx_count == 1U);
    assert(!Test_IsJ4310Special(&test_tx[0].frame, 0xFDU));
    assert(!Test_IsJ4310Special(&test_tx[0].frame, 0xFCU));
    assert(Test_FloatClose(Test_ReadJ4310MitPosition(&test_tx[0].frame),
                           -1.0f,
                           0.001f));

    j4310_mode_tick_ms = 102U;
    Test_CheckJ4310ReferenceModes(&j4310_cmd, &j4310_mode_tick_ms);

    j4310_cmd.mode = MOTOR_CMD_GLOBAL_STOP;
    Test_ResetTx();
    UpperMotorPort_BeginCycle(j4310_mode_tick_ms++);
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_J4310], &j4310_cmd, NULL));
    assert(test_tx_count == 1U);
    assert(Test_IsJ4310Special(&test_tx[0].frame, 0xFDU));

    j4310_cmd.mode = MOTOR_CMD_MIT;
    Test_ResetTx();
    UpperMotorPort_BeginCycle(j4310_mode_tick_ms);
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_J4310], &j4310_cmd, NULL));
    Test_FeedJ4310State(j4310_mode_tick_ms, 0x0BU);
    assert(UpperMotorPort_GetHealth(j4310_mode_tick_ms, &health));
    assert(UpperMotorPort_GetPendingFault(&fault));
    assert(fault.model == MOTOR_MODEL_J4310);
    assert(fault.can_bus == 1U);
    assert(fault.node_id == 0x06U);
    assert(fault.error_code == 0x0BU);
    assert(fault.tick_ms == j4310_mode_tick_ms);
    UpperMotorPort_MarkFaultSent(fault.sequence);
    assert(!UpperMotorPort_GetPendingFault(&fault));
    Test_ResetTx();
    UpperMotorPort_BeginCycle(j4310_mode_tick_ms);
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_J4310], &j4310_cmd, NULL));
    assert(test_tx_count == 1U);
    assert(Test_IsJ4310Special(&test_tx[0].frame, 0xFDU));

    Test_ResetTx();
    UpperMotorPort_BeginCycle(200U);
    assert(UpperMotorPort_Send(
        &upper_motor_cfg[UPPER_MOTOR_ARM_J4310], &j4310_cmd, NULL));
    assert(test_tx_count == 0U);

    Test_FeedJ4310(201U);
    UpperMotorPort_BeginCycle(201U);
    assert(UpperMotorPort_GetJ4310OutputPosition(1U,
                                                 0x06U,
                                                 &j4310_position_rad));
    assert(Test_FloatClose(j4310_position_rad, 12.5f, 0.001f));
    assert(UpperMotorPort_GetJ4310Feedback(1U,
                                           0x06U,
                                           &j4310_feedback));
    assert(Test_FloatClose(j4310_feedback.position_rad,
                            12.5f,
                            0.001f));
    assert(Test_FloatClose(j4310_feedback.torque_nm,
                            TEST_J4310_TORQUE_MAP_MAX_NM,
                            0.001f));
    assert(j4310_feedback.updated_at_ms == 201U);
    assert(j4310_feedback.state == 0x00U);
    UpperMotorPort_BeginCycle(252U);
    assert(!UpperMotorPort_GetJ4310OutputPosition(1U,
                                                   0x06U,
                                                   &j4310_position_rad));
    Test_ResetTx();
    assert(!UpperMotorPort_SaveJ4310Zero(1U, 0x06U));
    assert(test_tx_count == 0U);
    Test_FeedJ4310(253U);
    UpperMotorPort_BeginCycle(253U);
    assert(UpperMotorPort_SaveJ4310Zero(1U, 0x06U));
    assert(test_tx_count == 2U);
    assert(test_tx[0].can_bus == 1U);
    assert(test_tx[0].frame.id == 0x06U);
    assert(Test_IsJ4310Special(&test_tx[0].frame, 0xFDU));
    assert(Test_IsJ4310Special(&test_tx[1].frame, 0xFEU));
    Test_FeedJ4310State(255U, 0x01U);
    UpperMotorPort_BeginCycle(254U);
    assert(UpperMotorPort_GetJ4310OutputPosition(1U,
                                                 0x06U,
                                                 &j4310_position_rad));
    Test_CheckDjiReferenceControl();
    Test_CheckDjiRestartFeedbackCalibration();
    return 0;
}
