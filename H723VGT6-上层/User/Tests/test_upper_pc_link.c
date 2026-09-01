/**
 * @file test_upper_pc_link.c
 * @brief 验证上层 PC 链路的命令解析、会话和回传组帧。
 */

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "upper_pc_link.h"

/** 用于暂存该模块数据的缓冲区容量。 */
#define TEST_FRAME_CAPACITY  140U
/** 测试协议载荷中表示 J4310 型号的数值。 */
#define TEST_MOTOR_MODEL_J4310 0U
/** 测试协议载荷中表示 M2006 型号的数值。 */
#define TEST_MOTOR_MODEL_M2006 2U

static upper_pc_target_t test_target;
static bool test_target_received;
static bool test_estop_received;
static bool test_action_received;
static uint8_t test_action;
static uint8_t test_action_can_bus;
static uint8_t test_action_node_id;
static uint8_t test_action_value;
static bool test_aux_received;
static uint8_t test_aux_output_bits;
static uint8_t test_aux_update_mask;

/* 功能：向测试缓冲区写入小端 16 位整数；用途：手工构造上位机协议载荷；结果写入 data。 */
static void Test_WriteU16(uint8_t *data /* 待处理数据的首地址 */, uint16_t value /* 需要检查、限幅或编码的输入值 */)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

/* 功能：向测试缓冲区写入小端单精度浮点数；用途：手工构造运动与 PID 参数；结果写入 data。 */
static void Test_WriteFloat(uint8_t *data /* 待处理数据的首地址 */, float value /* 需要检查、限幅或编码的输入值 */)
{
    uint32_t bits;

    (void)memcpy(&bits, &value, sizeof(bits));
    data[0] = (uint8_t)bits;
    data[1] = (uint8_t)(bits >> 8U);
    data[2] = (uint8_t)(bits >> 16U);
    data[3] = (uint8_t)(bits >> 24U);
}

/* 功能：从测试帧读取小端单精度浮点数；用途：核对状态和遥测载荷；返回值表示解码结果。 */
static float Test_ReadFloat(const uint8_t *data /* 待处理数据的首地址 */)
{
    uint32_t bits;
    float value;

    bits = (uint32_t)data[0] |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
    (void)memcpy(&value, &bits, sizeof(value));
    return value;
}

/* 功能：从测试帧读取小端 32 位整数；用途：核对计数器和时间字段；返回值表示解码结果。 */
static uint32_t Test_ReadU32(const uint8_t *data /* 待处理数据的首地址 */)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

/* 功能：从测试帧中读取 16 位整数；用途：校验编码后的协议字段；返回值表示解码结果。 */
static uint16_t Test_ReadU16(const uint8_t *data /* 待处理数据的首地址 */)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

/* 功能：判断两个浮点数是否在误差容限内；用途：避免协议换算测试受舍入影响；返回 true 表示足够接近。 */
static bool Test_FloatClose(float actual /* 测试或判断使用的实际值 */, float expected /* 测试期望得到的参考值 */, float tolerance /* 比较实际值与期望值时允许的误差 */)
{
    float error = actual - expected;

    return (error >= -tolerance) && (error <= tolerance);
}

/* 功能：记录链路解码出的整机目标；用途：作为命令回调测试桩；无返回值表示目标快照与调用次数已保存。 */
static void Test_OnTarget(const upper_pc_target_t *target /* 本次需要应用的控制目标 */, void *user_data /* 调用回调函数时传递的用户上下文 */)
{
    (void)user_data;
    assert(target != NULL);
    test_target = *target;
    test_target_received = true;
}

/* 功能：记录急停回调是否被触发；用途：验证会话内急停消息分发；无返回值表示急停计数已更新。 */
static void Test_OnEStop(void *user_data /* 调用回调函数时传递的用户上下文 */)
{
    (void)user_data;
    test_estop_received = true;
}

/* 功能：记录电机维护动作回调参数；用途：验证保存零点命令的解码和路由；无返回值表示动作快照已保存。 */
static void Test_OnMotorAction(uint8_t action /* 需要执行的电机或遥控动作 */,
                               uint8_t can_bus /* CAN 总线编号 */,
                               uint8_t node_id /* 电机协议节点编号 */,
                               uint8_t value /* 需要检查、限幅或编码的输入值 */,
                               void *user_data /* 调用回调函数时传递的用户上下文 */)
{
    (void)user_data;
    test_action = action;
    test_action_can_bus = can_bus;
    test_action_node_id = node_id;
    test_action_value = value;
    test_action_received = true;
}

/* 功能：执行 OnAuxControl 场景测试；用途：验证对应输入下的行为和断言；无返回值表示由断言报告结果。 */
static void Test_OnAuxControl(uint8_t output_bits /* 需要写入辅助控制板的输出位 */,
                              uint8_t update_mask /* 指定本次允许改变哪些辅助输出位的掩码 */,
                              void *user_data /* 调用回调函数时传递的用户上下文 */)
{
    (void)user_data;
    test_aux_output_bits = output_bits;
    test_aux_update_mask = update_mask;
    test_aux_received = true;
}

/* 功能：构造基础上层控制命令测试帧；用途：覆盖速度模式及不同载荷长度；返回值表示编码后的帧长度。 */
static size_t Test_BuildCommand(uint16_t payload_size /* 待编码载荷的字节数 */, uint8_t *frame /* 需要解析或发送的 CAN 或协议帧 */)
{
    uint8_t payload[50] = {0};
    static const float value[8] =
    {
        1.0f,
        2.0f,
        3.0f,
        4.0f,
        5.0f,
        6.0f,
        7.0f,
        8.0f
    };
    size_t index;

    Test_WriteU16(payload, 0x0003U);
    for (index = 0U; index < 8U; index++)
    {
        Test_WriteFloat(&payload[2U + index * sizeof(float)], value[index]);
    }
    return PcProtocol_Encode(PC_MSG_UPPER_CMD,
                             17U,
                             payload,
                             payload_size,
                             frame,
                             TEST_FRAME_CAPACITY);
}

/* 功能：构造指定序号的握手测试帧；用途：建立并验证链路会话；返回值表示编码后的帧长度。 */
static size_t Test_BuildHandshake(uint16_t sequence /* 用于匹配请求和响应的消息序号 */, uint8_t *frame /* 需要解析或发送的 CAN 或协议帧 */)
{
    static const uint8_t magic[UPPER_PC_HANDSHAKE_PAYLOAD_SIZE] =
    {
        'H', '7', '2', '3'
    };

    return PcProtocol_Encode(PC_MSG_HANDSHAKE,
                             sequence,
                             magic,
                             sizeof(magic),
                             frame,
                             TEST_FRAME_CAPACITY);
}

/* 功能：构造标准位置控制测试帧；用途：验证各机构位置目标解码；返回值表示编码后的帧长度。 */
static size_t Test_BuildPositionCommand(uint8_t *frame /* 需要解析或发送的 CAN 或协议帧 */)
{
    uint8_t payload[UPPER_PC_POSITION_CMD_PAYLOAD_SIZE] = {0};
    static const float value[8] =
    {
        0.10f,
        0.20f,
        3.0f,
        0.2f,
        1.57f,
        -1.57f,
        0.78f,
        -0.78f
    };
    size_t index;

    Test_WriteU16(payload, 0x0002U);
    for (index = 0U; index < 8U; index++)
    {
        Test_WriteFloat(&payload[2U + index * sizeof(float)], value[index]);
    }
    return PcProtocol_Encode(PC_MSG_UPPER_POSITION_CMD,
                             18U,
                             payload,
                             sizeof(payload),
                             frame,
                             TEST_FRAME_CAPACITY);
}

/* 功能：构造含 PID 参数的扩展位置命令；用途：验证 GUI 单位换算和参数下发；返回值表示编码后的帧长度。 */
static size_t Test_BuildExtendedPositionCommand(uint16_t enable_mask /* 控制命令中允许启用的机构位图 */,
                                                 uint8_t *frame /* 需要解析或发送的 CAN 或协议帧 */)
{
    uint8_t payload[UPPER_PC_EXTENDED_POSITION_CMD_PAYLOAD_SIZE] = {0};
    static const float value[30] =
    {
        0.10f, 0.20f, 3.0f, 0.2f, 0.5f, 8.0f,
        1.57f, -1.57f, 0.78f, -0.78f,
        79.5678059799f, 96.7271573771f, 0.0f, 14.022502658f, 16384.0f,
        63.7950221167f, 38.2228810547f, 3.61399672504f,
        0.00290389972619f, 66.3985601018f,
        350.0f, 250.0f, 0.0f, 0.5f, 10000.0f,
        900.0f, 500.0f, 5.0f, 0.002f, 50.0f
    };
    size_t index;

    Test_WriteU16(payload, enable_mask);
    for (index = 0U; index < 30U; index++)
    {
        Test_WriteFloat(&payload[2U + index * sizeof(float)], value[index]);
    }
    return PcProtocol_Encode(PC_MSG_UPPER_POSITION_CMD,
                             19U,
                             payload,
                             sizeof(payload),
                             frame,
                             TEST_FRAME_CAPACITY);
}

/* 功能：构造含夹持转矩与限制的位置命令；用途：验证 J4310 转矩字段解码和使能组合；返回值表示编码后的帧长度。 */
static size_t Test_BuildPositionTorqueCommand(uint16_t enable_mask /* 控制命令中允许启用的机构位图 */,
                                               uint8_t *frame /* 需要解析或发送的 CAN 或协议帧 */)
{
    uint8_t payload[UPPER_PC_POSITION_TORQUE_CMD_PAYLOAD_SIZE] = {0};
    static const float value[10] =
    {
        0.10f, 0.20f, 3.0f, 0.2f, 0.5f, 8.0f,
        1.57f, -1.57f, 0.78f, -0.78f
    };
    size_t index;

    Test_WriteU16(payload, enable_mask);
    for (index = 0U; index < 10U; index++)
    {
        Test_WriteFloat(&payload[2U + index * sizeof(float)], value[index]);
    }
    return PcProtocol_Encode(PC_MSG_UPPER_POSITION_CMD,
                             24U,
                             payload,
                             sizeof(payload),
                             frame,
                             TEST_FRAME_CAPACITY);
}

/* 功能：运行上位机链路协议的全部主机断言；用途：检查握手、命令、超时、事件和遥测编码；返回 0 表示全部通过。 */
int main(void)
{
    upper_pc_link_t link;
    uint8_t frame[TEST_FRAME_CAPACITY];
    size_t frame_size;

    UpperPcLink_Init(&link,
                     Test_OnTarget,
                     Test_OnEStop,
                     Test_OnMotorAction,
                     NULL);
    UpperPcLink_SetAuxControlHandler(&link, Test_OnAuxControl);
    assert(!UpperPcLink_IsSessionActive(&link, 0U));

    frame_size = PcProtocol_Encode(PC_MSG_FLASH_INFO_REQUEST,
                                   8U,
                                   NULL,
                                   0U,
                                   frame,
                                   TEST_FRAME_CAPACITY);
    assert(frame_size > 0U);
    UpperPcLink_Push(&link, frame, frame_size, 49U);
    assert(UpperPcLink_HasFlashInfoPending(&link));
    assert(UpperPcLink_GetFlashInfoSequence(&link) == 8U);
    frame_size = UpperPcLink_BuildFlashInfo(&link,
                                            2U,
                                            false,
                                            0U,
                                            0U,
                                            0U,
                                            0U,
                                            4096UL,
                                            frame,
                                            TEST_FRAME_CAPACITY);
    assert(frame_size > 0U);
    assert(frame[3] == PC_MSG_FLASH_INFO);
    assert(Test_ReadU16(&frame[4]) == 8U);
    assert(frame[8] == 2U);
    assert(frame[9] == 0U);
    UpperPcLink_MarkFlashInfoSent(&link, 8U);
    assert(!UpperPcLink_HasFlashInfoPending(&link));
    assert(link.last_rx_tick_ms == 0U);

    frame_size = Test_BuildCommand(UPPER_PC_CMD_PAYLOAD_SIZE, frame);
    assert(frame_size > 0U);
    UpperPcLink_Push(&link, frame, frame_size, 50U);
    assert(!test_target_received);
    assert(link.last_rx_tick_ms == 0U);

    frame_size = Test_BuildHandshake(9U, frame);
    assert(frame_size > 0U);
    UpperPcLink_Push(&link, frame, frame_size, 99U);
    assert(UpperPcLink_HasHandshakePending(&link));
    assert(link.handshake_received_count == 1U);
    assert(link.handshake_received_tick_ms == 99U);
    assert(!UpperPcLink_IsHandshakeAckDue(&link, 118U, 20U));
    assert(UpperPcLink_IsHandshakeAckDue(&link, 119U, 20U));
    assert(UpperPcLink_GetHandshakeSequence(&link) == 9U);
    frame_size = UpperPcLink_BuildHandshakeAck(&link, frame, TEST_FRAME_CAPACITY);
    assert(frame_size > 0U);
    assert(frame[3] == PC_MSG_ACK);
    assert(frame[4] == 9U);
    assert(frame[5] == 0U);
    assert(frame[8] == 'H');
    assert(frame[9] == '7');
    assert(frame[10] == '2');
    assert(frame[11] == '3');
    UpperPcLink_MarkHandshakeAckSent(&link, 9U);
    assert(!UpperPcLink_HasHandshakePending(&link));
    assert(UpperPcLink_IsSessionActive(&link, 99U));

    frame_size = PcProtocol_Encode(PC_MSG_FLASH_INFO_REQUEST,
                                   10U,
                                   NULL,
                                   0U,
                                   frame,
                                   TEST_FRAME_CAPACITY);
    assert(frame_size > 0U);
    UpperPcLink_Push(&link, frame, frame_size, 100U);
    assert(UpperPcLink_HasFlashInfoPending(&link));
    assert(UpperPcLink_GetFlashInfoSequence(&link) == 10U);
    assert(link.last_rx_sequence == 9U);
    assert(link.last_rx_tick_ms == 99U);
    frame_size = UpperPcLink_BuildFlashInfo(&link,
                                            0U,
                                            true,
                                            0xEF4015UL,
                                            2048UL,
                                            512UL,
                                            256U,
                                            4096UL,
                                            frame,
                                            TEST_FRAME_CAPACITY);
    assert(frame_size > 0U);
    assert(frame[3] == PC_MSG_FLASH_INFO);
    assert(Test_ReadU16(&frame[4]) == 10U);
    assert(Test_ReadU16(&frame[6]) == UPPER_PC_FLASH_INFO_PAYLOAD_SIZE);
    assert(frame[8] == 0U);
    assert(frame[9] == 1U);
    assert(Test_ReadU32(&frame[10]) == 0xEF4015UL);
    assert(Test_ReadU32(&frame[14]) == 2048UL);
    assert(Test_ReadU32(&frame[18]) == 512UL);
    assert(Test_ReadU16(&frame[22]) == 256U);
    assert(Test_ReadU32(&frame[24]) == 4096UL);
    UpperPcLink_MarkFlashInfoSent(&link, 10U);
    assert(!UpperPcLink_HasFlashInfoPending(&link));

    {
        const uint8_t aux_payload[UPPER_PC_AUX_CONTROL_PAYLOAD_SIZE] = {
            0x0BU, 0x04U
        };

        frame_size = PcProtocol_Encode(PC_MSG_AUX_CONTROL,
                                       11U,
                                       aux_payload,
                                       sizeof(aux_payload),
                                       frame,
                                       TEST_FRAME_CAPACITY);
        assert(frame_size > 0U);
        UpperPcLink_Push(&link, frame, frame_size, 100U);
        assert(test_aux_received);
        assert(test_aux_output_bits == 0x0BU);
        assert(test_aux_update_mask == 0x04U);
    }

    {
        const uint8_t legacy_aux_payload[UPPER_PC_AUX_CONTROL_PAYLOAD_SIZE] = {
            0x05U, 0x00U
        };

        test_aux_received = false;
        frame_size = PcProtocol_Encode(PC_MSG_AUX_CONTROL,
                                       12U,
                                       legacy_aux_payload,
                                       sizeof(legacy_aux_payload),
                                       frame,
                                       TEST_FRAME_CAPACITY);
        assert(frame_size > 0U);
        UpperPcLink_Push(&link, frame, frame_size, 100U);
        assert(test_aux_received);
        assert(test_aux_output_bits == 0x05U);
        assert(test_aux_update_mask == 0x00U);
    }

    frame_size = Test_BuildCommand(UPPER_PC_CMD_PAYLOAD_SIZE, frame);
    assert(frame_size > 0U);
    UpperPcLink_Push(&link, frame, frame_size, 100U);
    assert(test_target_received);
    assert(link.last_rx_sequence == 17U);
    assert(link.last_rx_tick_ms == 100U);
    assert(test_target.arm.enabled);
    assert(test_target.arm.j4310_commanded);
    assert(test_target.arm.m3508_enabled);
    assert(test_target.gate.enabled);
    assert(!test_target.gripper.enabled);
    assert(test_target.arm.grip_pos_rad == 1.0f);
    assert(test_target.arm.grip_vel_rad_s == 2.0f);
    assert(test_target.arm.grip_kp == 3.0f);
    assert(test_target.arm.grip_kd == 4.0f);
    assert(test_target.arm.m3508_vel_rad_s[0] == 5.0f);
    assert(test_target.arm.m3508_vel_rad_s[1] == 6.0f);
    assert(test_target.gate.m2006_vel_rad_s == 7.0f);
    assert(test_target.gripper.m2006_vel_rad_s == 8.0f);

    test_target_received = false;
    frame_size = Test_BuildCommand(42U, frame);
    assert(frame_size > 0U);
    UpperPcLink_Push(&link, frame, frame_size, 101U);
    assert(!test_target_received);
    assert(link.command_error_count == 1U);

    frame_size = Test_BuildCommand(UPPER_PC_CMD_PAYLOAD_SIZE, frame);
    assert(frame_size > 0U);
    frame[2] = 2U;
    UpperPcLink_Push(&link, frame, frame_size, 102U);
    assert(!test_target_received);
    assert(link.last_rx_tick_ms == 100U);

    frame_size = Test_BuildPositionCommand(frame);
    assert(frame_size > 0U);
    UpperPcLink_Push(&link, frame, frame_size, 103U);
    assert(test_target_received);
    assert(test_target.position_mode);
    assert(test_target.arm.position_mode);
    assert(test_target.arm.m3508_pos_rad[0] == 1.57f);
    assert(test_target.arm.m3508_pos_rad[1] == -1.57f);
    assert(test_target.gate.m2006_pos_rad == 0.78f);
    assert(test_target.gripper.m2006_pos_rad == -0.78f);
    assert(link.last_rx_sequence == 18U);

    test_target_received = false;
    frame_size = Test_BuildExtendedPositionCommand(0x0002U, frame);
    assert(frame_size > 0U);
    UpperPcLink_Push(&link, frame, frame_size, 104U);
    assert(test_target_received);
    assert(!test_target.arm.pid_update);
    assert(test_target.gate.pid_update);
    assert(!test_target.gripper.pid_update);
    assert(test_target.arm.grip_torque_nm == 0.5f);
    assert(test_target.arm.grip_torque_limit_nm == 8.0f);
    assert(test_target.arm.m3508_pos_rad[0] == 1.57f);
    assert(test_target.arm.m3508_pos_rad[1] == -1.57f);
    assert(test_target.gate.m2006_pos_rad == 0.78f);
    assert(test_target.gripper.m2006_pos_rad == -0.78f);
    assert(Test_FloatClose(test_target.gate.m2006_speed_pid.kp,
                           3.34225380f,
                           0.000001f));
    assert(Test_FloatClose(test_target.gate.m2006_speed_pid.ki,
                           2.38732415f,
                           0.000001f));
    assert(test_target.gate.m2006_speed_pid.kd == 0.0f);
    assert(Test_FloatClose(
        test_target.gate.m2006_speed_pid.integral_limit,
        0.05235988f,
        0.000001f));
    assert(test_target.gate.m2006_speed_pid.output_limit == 10.0f);
    assert(Test_FloatClose(test_target.gate.m2006_position_pid.kp,
                           94.24777961f,
                           0.00001f));
    assert(Test_FloatClose(test_target.gate.m2006_position_pid.ki,
                           52.35987756f,
                           0.00001f));
    assert(Test_FloatClose(test_target.gate.m2006_position_pid.kd,
                           0.52359878f,
                           0.000001f));
    assert(test_target.gate.m2006_position_pid.integral_limit == 0.002f);
    assert(Test_FloatClose(
        test_target.gate.m2006_position_pid.output_limit,
        5.23598776f,
        0.000001f));
    assert(link.last_rx_sequence == 19U);

    test_target_received = false;
    frame_size = Test_BuildPositionTorqueCommand(0x0002U, frame);
    assert(frame_size > 0U);
    UpperPcLink_Push(&link, frame, frame_size, 105U);
    assert(test_target_received);
    assert(test_target.gate.enabled);
    assert(!test_target.gripper.enabled);
    assert(!test_target.gate.pid_update);
    assert(test_target.arm.grip_torque_nm == 0.5f);

    test_target_received = false;
    frame_size = Test_BuildPositionTorqueCommand(0x0006U, frame);
    assert(frame_size > 0U);
    UpperPcLink_Push(&link, frame, frame_size, 106U);
    assert(test_target_received);
    assert(test_target.gate.enabled);
    assert(test_target.gripper.enabled);
    assert(!test_target.gate.pid_update);
    assert(!test_target.gripper.pid_update);

    test_target_received = false;
    frame_size = Test_BuildExtendedPositionCommand(0x0006U, frame);
    assert(frame_size > 0U);
    UpperPcLink_Push(&link, frame, frame_size, 107U);
    assert(!test_target_received);
    assert(link.command_error_count == 2U);

    {
        uint8_t payload[UPPER_PC_POSITION_CMD_PAYLOAD_SIZE] = {0};

        Test_WriteU16(payload, 0x0008U);
        frame_size = PcProtocol_Encode(PC_MSG_UPPER_POSITION_CMD,
                                       20U,
                                       payload,
                                       sizeof(payload),
                                       frame,
                                       TEST_FRAME_CAPACITY);
        assert(frame_size > 0U);
        UpperPcLink_Push(&link, frame, frame_size, 105U);
        assert(!test_target.arm.enabled);
        assert(test_target.arm.m3508_enabled);
        assert(!test_target.gate.enabled);
        assert(!test_target.gripper.enabled);
    }

    {
        uint8_t payload[UPPER_PC_POSITION_CMD_PAYLOAD_SIZE] = {0};

        Test_WriteU16(payload, 0x0010U);
        frame_size = PcProtocol_Encode(PC_MSG_UPPER_POSITION_CMD,
                                       21U,
                                       payload,
                                       sizeof(payload),
                                       frame,
                                       TEST_FRAME_CAPACITY);
        assert(frame_size > 0U);
        UpperPcLink_Push(&link, frame, frame_size, 106U);
        assert(test_target.arm.enabled);
        assert(test_target.arm.j4310_commanded);
        assert(!test_target.arm.m3508_enabled);
        assert(!test_target.gate.enabled);
        assert(!test_target.gripper.enabled);
    }

    {
        uint8_t payload[UPPER_PC_POSITION_CMD_PAYLOAD_SIZE] = {0};

        Test_WriteU16(payload, 0x0020U);
        frame_size = PcProtocol_Encode(PC_MSG_UPPER_POSITION_CMD,
                                       22U,
                                       payload,
                                       sizeof(payload),
                                       frame,
                                       TEST_FRAME_CAPACITY);
        assert(frame_size > 0U);
        UpperPcLink_Push(&link, frame, frame_size, 107U);
        assert(!test_target.arm.enabled);
        assert(test_target.arm.m3508_enabled);
        assert(!test_target.arm.j4310_commanded);
        assert(!test_target.gate.enabled);
        assert(!test_target.gripper.enabled);
    }

    {
        uint8_t payload[UPPER_PC_POSITION_CMD_PAYLOAD_SIZE] = {0};

        Test_WriteU16(payload, 0x0040U);
        frame_size = PcProtocol_Encode(PC_MSG_UPPER_POSITION_CMD,
                                       23U,
                                       payload,
                                       sizeof(payload),
                                       frame,
                                       TEST_FRAME_CAPACITY);
        assert(frame_size > 0U);
        UpperPcLink_Push(&link, frame, frame_size, 108U);
        assert(!test_target.arm.enabled);
        assert(!test_target.arm.m3508_enabled);
        assert(test_target.arm.j4310_commanded);
    }

    {
        const uint8_t action_payload[UPPER_PC_MOTOR_ACTION_PAYLOAD_SIZE] =
        {
            UPPER_PC_ACTION_J4310_SAVE_ZERO, 1U, 0x06U
        };
        frame_size = PcProtocol_Encode(PC_MSG_MOTOR_ACTION,
                                       23U,
                                       action_payload,
                                       sizeof(action_payload),
                                       frame,
                                       TEST_FRAME_CAPACITY);
        assert(frame_size > 0U);
        UpperPcLink_Push(&link, frame, frame_size, 108U);
        assert(test_action_received);
        assert(test_action == UPPER_PC_ACTION_J4310_SAVE_ZERO);
        assert(test_action_can_bus == 1U);
        assert(test_action_node_id == 0x06U);
        assert(test_action_value == 0U);
    }

    {
        const uint8_t action_payload[UPPER_PC_MOTOR_ACTION_PAYLOAD_SIZE] =
        {
            UPPER_PC_ACTION_J4310_ENABLE, 1U, 0x06U
        };
        test_action_received = false;
        frame_size = PcProtocol_Encode(PC_MSG_MOTOR_ACTION,
                                       24U,
                                       action_payload,
                                       sizeof(action_payload),
                                       frame,
                                       TEST_FRAME_CAPACITY);
        assert(frame_size > 0U);
        UpperPcLink_Push(&link, frame, frame_size, 109U);
        assert(test_action_received);
        assert(test_action == UPPER_PC_ACTION_J4310_ENABLE);
        assert(test_action_can_bus == 1U);
        assert(test_action_node_id == 0x06U);
        assert(test_action_value == 0U);
    }

    {
        const uint8_t action_payload[
            UPPER_PC_MOTOR_CONFIG_ACTION_PAYLOAD_SIZE] =
        {
            UPPER_PC_ACTION_J4310_AUTO_RETURN, 1U, 0x06U, 1U
        };
        test_action_received = false;
        frame_size = PcProtocol_Encode(PC_MSG_MOTOR_ACTION,
                                       24U,
                                       action_payload,
                                       sizeof(action_payload),
                                       frame,
                                       TEST_FRAME_CAPACITY);
        assert(frame_size > 0U);
        UpperPcLink_Push(&link, frame, frame_size, 109U);
        assert(test_action_received);
        assert(test_action == UPPER_PC_ACTION_J4310_AUTO_RETURN);
        assert(test_action_can_bus == 1U);
        assert(test_action_node_id == 0x06U);
        assert(test_action_value == 1U);
    }

    frame_size = UpperPcLink_BuildMotorActionResult(
                     &link,
                     UPPER_PC_ACTION_J4310_SAVE_ZERO,
                     1U,
                     0x06U,
                     0U,
                     108U,
                     frame,
                     TEST_FRAME_CAPACITY);
    assert(frame_size > 0U);
    assert(frame[3] == PC_MSG_MOTOR_ACTION_RESULT);
    assert(frame[8] == UPPER_PC_ACTION_J4310_SAVE_ZERO);
    assert(frame[9] == 1U);
    assert(frame[10] == 0x06U);
    assert(frame[11] == 0U);

    frame_size = UpperPcLink_BuildMotorFault(&link,
                                              TEST_MOTOR_MODEL_J4310,
                                              1U,
                                              0x06U,
                                              0x0BU,
                                              109U,
                                              frame,
                                              TEST_FRAME_CAPACITY);
    assert(frame_size > 0U);
    assert(frame[3] == PC_MSG_FAULT);
    assert(frame[8] == TEST_MOTOR_MODEL_J4310);
    assert(frame[9] == 1U);
    assert(frame[10] == 0x06U);
    assert(frame[11] == 0x0BU);

    frame_size = UpperPcLink_BuildState(
        &link,
        &(upper_pc_state_t)
        {
            .j4310_position_valid = true,
            .j4310_position_rad = 1.25f,
            .j4310_bus_rx_frames = 123U,
            .j4310_rx_valid = true,
            .j4310_rx =
            {
                .accepted_frames = 100U,
                .rejected_format_frames = 4U,
                .rejected_master_id_frames = 5U,
                .rejected_feedback_id_frames = 6U,
                .last_can_id = 0x016U,
                .last_dlc = 8U,
                .last_data0 = 0x06U,
                .last_result = 1U
            },
            .j4310_tx_valid = true,
            .j4310_tx =
            {
                .attempted_frames = 120U,
                .queued_frames = 118U,
                .failed_frames = 2U,
                .enable_frames = 5U,
                .mit_frames = 110U,
                .disable_frames = 3U,
                .last_can_id = 0x006U,
                .last_dlc = 8U,
                .last_data7 = 0xFCU,
                .enable_confirmed = true,
                .feedback_state = 0x01U
            },
            .j4310_auto_return =
            {
                .available = true,
                .enabled = true,
                .active = true,
                .stage = 2U
            }
        },
        110U,
        frame,
        TEST_FRAME_CAPACITY);
    assert(frame_size > 0U);
    assert(frame[3] == PC_MSG_ROBOT_STATE);
    assert(frame[6] == 84U);
    assert(frame[7] == 0U);
    assert(Test_FloatClose(Test_ReadFloat(&frame[28]), 1.25f, 0.000001f));
    assert(frame[32] == 1U);
    assert(Test_ReadU32(&frame[33]) == 123U);
    assert(Test_ReadU32(&frame[37]) == 100U);
    assert(Test_ReadU32(&frame[41]) == 4U);
    assert(Test_ReadU32(&frame[45]) == 5U);
    assert(Test_ReadU32(&frame[49]) == 6U);
    assert(frame[53] == 0x16U);
    assert(frame[54] == 0x00U);
    assert(frame[55] == 8U);
    assert(frame[56] == 0x06U);
    assert(frame[57] == 1U);
    assert(Test_ReadU32(&frame[58]) == 120U);
    assert(Test_ReadU32(&frame[62]) == 118U);
    assert(Test_ReadU32(&frame[66]) == 2U);
    assert(Test_ReadU32(&frame[70]) == 5U);
    assert(Test_ReadU32(&frame[74]) == 110U);
    assert(Test_ReadU32(&frame[78]) == 3U);
    assert(frame[82] == 0x06U);
    assert(frame[83] == 0x00U);
    assert(frame[84] == 8U);
    assert(frame[85] == 0xFCU);
    assert(frame[86] == 1U);
    assert(frame[87] == 0x01U);
    assert(frame[88] == 1U);
    assert(frame[89] == 1U);
    assert(frame[90] == 1U);
    assert(frame[91] == 2U);

    frame_size = UpperPcLink_BuildDjiTelemetry(
                     &link,
                     &(upper_pc_dji_telemetry_t)
                     {
                         TEST_MOTOR_MODEL_M2006,
                         3U,
                         1U,
                         true,
                         true,
                         true,
                         4.0f,
                         3.0f,
                         1.0f
                     },
                     (uint32_t[]){11U, 22U, 33U},
                     frame,
                     TEST_FRAME_CAPACITY);
    assert(frame_size > 0U);
    assert(frame[3] == PC_MSG_DJI_TELEMETRY);
    assert(frame[6] == 28U);
    assert(frame[7] == 0U);
    assert(frame[8] == TEST_MOTOR_MODEL_M2006);
    assert(frame[9] == 3U);
    assert(frame[10] == 1U);
    assert(frame[11] == 0x07U);
    assert(Test_FloatClose(Test_ReadFloat(&frame[12]), 4.0f, 0.000001f));
    assert(Test_FloatClose(Test_ReadFloat(&frame[16]), 3.0f, 0.000001f));
    assert(Test_FloatClose(Test_ReadFloat(&frame[20]), 1.0f, 0.000001f));
    assert(Test_ReadU32(&frame[24]) == 11U);
    assert(Test_ReadU32(&frame[28]) == 22U);
    assert(Test_ReadU32(&frame[32]) == 33U);

    assert(UpperPcLink_IsSessionActive(&link, 309U));
    assert(UpperPcLink_IsSessionActive(&link, 310U));
    assert(!UpperPcLink_IsTimedOut(&link, 310U));
    assert(link.remote_active);

    frame_size = PcProtocol_Encode(PC_MSG_HEARTBEAT,
                                   24U,
                                   NULL,
                                   0U,
                                   frame,
                                   TEST_FRAME_CAPACITY);
    assert(frame_size > 0U);
    UpperPcLink_Push(&link, frame, frame_size, 311U);
    assert(UpperPcLink_IsSessionActive(&link, 311U));
    assert(link.last_rx_tick_ms == 311U);

    frame_size = Test_BuildHandshake(25U, frame);
    assert(frame_size > 0U);
    UpperPcLink_Push(&link, frame, frame_size, 312U);
    assert(!UpperPcLink_IsSessionActive(&link, 312U));
    UpperPcLink_MarkHandshakeAckSent(&link, 25U);
    assert(UpperPcLink_IsSessionActive(&link, 312U));

    test_target_received = false;
    frame_size = Test_BuildCommand(UPPER_PC_CMD_PAYLOAD_SIZE, frame);
    assert(frame_size > 0U);
    UpperPcLink_Push(&link, frame, frame_size, 312U);
    assert(test_target_received);
    assert(link.remote_active);

    frame_size = Test_BuildHandshake(26U, frame);
    assert(frame_size > 0U);
    UpperPcLink_Push(&link, frame, frame_size, 513U);
    assert(link.remote_active);
    assert(!UpperPcLink_IsSessionActive(&link, 513U));
    UpperPcLink_MarkHandshakeAckSent(&link, 26U);
    assert(UpperPcLink_IsSessionActive(&link, 513U));
    assert(!UpperPcLink_IsTimedOut(&link, 513U));
    assert(UpperPcLink_IsSessionActive(&link, 513U));

    {
        const uint8_t estop_payload = 1U;

        frame_size = PcProtocol_Encode(PC_MSG_ESTOP,
                                       27U,
                                       &estop_payload,
                                       sizeof(estop_payload),
                                       frame,
                                       TEST_FRAME_CAPACITY);
        assert(frame_size > 0U);
        UpperPcLink_Push(&link, frame, frame_size, 514U);
        assert(test_estop_received);
        assert(!link.remote_active);
        assert(UpperPcLink_IsSessionActive(&link, 514U));
    }

    test_target_received = false;
    frame_size = Test_BuildCommand(UPPER_PC_CMD_PAYLOAD_SIZE, frame);
    assert(frame_size > 0U);
    UpperPcLink_Push(&link, frame, frame_size, 515U);
    assert(test_target_received);
    assert(link.remote_active);

    frame_size = Test_BuildHandshake(28U, frame);
    assert(frame_size > 0U);
    UpperPcLink_Push(&link, frame, frame_size, 516U);
    assert(!UpperPcLink_IsSessionActive(&link, 516U));
    UpperPcLink_MarkHandshakeAckSent(&link, 28U);
    assert(UpperPcLink_IsSessionActive(&link, 516U));
    return 0;
}
