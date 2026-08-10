#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "upper_pc_link.h"

#define TEST_FRAME_CAPACITY  140U

static upper_target_t test_target;
static bool test_target_received;

static void Test_WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static void Test_WriteFloat(uint8_t *data, float value)
{
    uint32_t bits;

    (void)memcpy(&bits, &value, sizeof(bits));
    data[0] = (uint8_t)bits;
    data[1] = (uint8_t)(bits >> 8U);
    data[2] = (uint8_t)(bits >> 16U);
    data[3] = (uint8_t)(bits >> 24U);
}

static void Test_OnTarget(const upper_target_t *target, void *user_data)
{
    (void)user_data;
    assert(target != NULL);
    test_target = *target;
    test_target_received = true;
}

static size_t Test_BuildCommand(uint16_t payload_size, uint8_t *frame)
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

    Test_WriteU16(payload, 0x0007U);
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

static size_t Test_BuildHandshake(uint16_t sequence, uint8_t *frame)
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

static size_t Test_BuildPositionCommand(uint8_t *frame)
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

    Test_WriteU16(payload, 0x0007U);
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

static size_t Test_BuildExtendedPositionCommand(uint8_t *frame)
{
    uint8_t payload[UPPER_PC_EXTENDED_POSITION_CMD_PAYLOAD_SIZE] = {0};
    static const float value[30] =
    {
        0.10f, 0.20f, 3.0f, 0.2f, 0.5f, 8.0f,
        1.57f, -1.57f, 0.78f, -0.78f,
        80.0f, 40.0f, 0.0f, 100.0f, 16384.0f,
        60.0f, 0.0f, 0.0f, 0.0f, 300.0f,
        350.0f, 250.0f, 0.0f, 0.5f, 10000.0f,
        220.0f, 500.0f, 5.0f, 0.002f, 50.0f
    };
    size_t index;

    Test_WriteU16(payload, 0x0007U);
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

int main(void)
{
    upper_pc_link_t link;
    uint8_t frame[TEST_FRAME_CAPACITY];
    size_t frame_size;

    UpperPcLink_Init(&link, Test_OnTarget, NULL, NULL);
    frame_size = Test_BuildHandshake(9U, frame);
    assert(frame_size > 0U);
    UpperPcLink_Push(&link, frame, frame_size, 99U);
    assert(UpperPcLink_HasHandshakePending(&link));
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

    frame_size = Test_BuildCommand(UPPER_PC_CMD_PAYLOAD_SIZE, frame);
    assert(frame_size > 0U);
    UpperPcLink_Push(&link, frame, frame_size, 100U);
    assert(test_target_received);
    assert(link.last_rx_sequence == 17U);
    assert(link.last_rx_tick_ms == 100U);
    assert(test_target.arm.enabled);
    assert(test_target.conveyor.enabled);
    assert(test_target.gripper.enabled);
    assert(test_target.arm.grip_pos_rad == 1.0f);
    assert(test_target.arm.grip_vel_rad_s == 2.0f);
    assert(test_target.arm.grip_kp == 3.0f);
    assert(test_target.arm.grip_kd == 4.0f);
    assert(test_target.arm.m3508_vel_rad_s[0] == 5.0f);
    assert(test_target.arm.m3508_vel_rad_s[1] == 6.0f);
    assert(test_target.conveyor.m2006_vel_rad_s == 7.0f);
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
    assert(test_target.conveyor.m2006_pos_rad == 0.78f);
    assert(test_target.gripper.m2006_pos_rad == -0.78f);
    assert(link.last_rx_sequence == 18U);

    test_target_received = false;
    frame_size = Test_BuildExtendedPositionCommand(frame);
    assert(frame_size > 0U);
    UpperPcLink_Push(&link, frame, frame_size, 104U);
    assert(test_target_received);
    assert(test_target.arm.pid_update);
    assert(test_target.arm.grip_torque_nm == 0.5f);
    assert(test_target.arm.grip_torque_limit_nm == 8.0f);
    assert(test_target.arm.m3508_speed_pid.kp > 0.0f);
    assert(test_target.arm.m3508_position_pid.output_limit > 0.0f);
    assert(test_target.conveyor.m2006_speed_pid.kp > 0.0f);
    assert(test_target.gripper.m2006_position_pid.output_limit > 0.0f);
    assert(link.last_rx_sequence == 19U);
    return 0;
}
