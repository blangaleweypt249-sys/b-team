#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "upper_pc_link.h"

#define TEST_FRAME_CAPACITY  80U

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
    static const float value[10] =
    {
        1.0f,
        2.0f,
        3.0f,
        4.0f,
        5.0f,
        6.0f,
        7.0f,
        8.0f,
        9.0f,
        10.0f
    };
    size_t index;

    Test_WriteU16(payload, 0x0007U);
    for (index = 0U; index < 10U; index++)
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

int main(void)
{
    upper_pc_link_t link;
    uint8_t frame[TEST_FRAME_CAPACITY];
    size_t frame_size;

    UpperPcLink_Init(&link, Test_OnTarget, NULL, NULL);
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
    assert(test_target.arm.m3508_vel_rad_s[3] == 8.0f);
    assert(test_target.conveyor.m2006_vel_rad_s == 9.0f);
    assert(test_target.gripper.m2006_vel_rad_s == 10.0f);

    test_target_received = false;
    frame_size = Test_BuildCommand(50U, frame);
    assert(frame_size > 0U);
    UpperPcLink_Push(&link, frame, frame_size, 101U);
    assert(!test_target_received);
    assert(link.command_error_count == 1U);

    frame_size = Test_BuildCommand(UPPER_PC_CMD_PAYLOAD_SIZE, frame);
    assert(frame_size > 0U);
    frame[2] = 1U;
    UpperPcLink_Push(&link, frame, frame_size, 102U);
    assert(!test_target_received);
    assert(link.last_rx_tick_ms == 100U);
    return 0;
}
