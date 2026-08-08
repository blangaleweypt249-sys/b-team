#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "bsp_can.h"
#include "upper_config.h"
#include "upper_motor_port.h"

#define TEST_TX_CAPACITY  16U

typedef struct
{
    uint8_t can_bus;
    can_frame_t frame;
} test_tx_t;

static test_tx_t test_tx[TEST_TX_CAPACITY];
static size_t test_tx_count;

bool BspCan_Send(uint8_t can_bus, const can_frame_t *frame)
{
    assert(frame != NULL);
    assert(test_tx_count < TEST_TX_CAPACITY);
    test_tx[test_tx_count].can_bus = can_bus;
    test_tx[test_tx_count].frame = *frame;
    test_tx_count++;
    return true;
}

static int16_t Test_ReadI16Be(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

static void Test_ResetTx(void)
{
    (void)memset(test_tx, 0, sizeof(test_tx));
    test_tx_count = 0U;
}

static void Test_FeedDji(uint8_t can_bus,
                         uint8_t node_id,
                         uint32_t tick_ms)
{
    can_frame_t frame = {0};

    frame.id = 0x200U + node_id;
    frame.dlc = 8U;
    UpperMotorPort_OnFrame(can_bus, &frame, tick_ms);
}

static void Test_FeedJ4310(uint32_t tick_ms)
{
    can_frame_t frame = {0};

    frame.id = 0U;
    frame.dlc = 8U;
    frame.data[0] = 3U;
    UpperMotorPort_OnFrame(1U, &frame, tick_ms);
}

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

static void Test_CheckTopology(void)
{
    assert(UPPER_CONTROL_FREQUENCY_HZ == 1000U);
    assert(upper_motor_cfg[UPPER_MOTOR_ARM_M3508_1].can_bus == 2U);
    assert(upper_motor_cfg[UPPER_MOTOR_ARM_M3508_1].node_id == 1U);
    assert(upper_motor_cfg[UPPER_MOTOR_ARM_M3508_2].can_bus == 2U);
    assert(upper_motor_cfg[UPPER_MOTOR_ARM_M3508_2].node_id == 2U);
    assert(upper_motor_cfg[UPPER_MOTOR_ARM_J4310].can_bus == 1U);
    assert(upper_motor_cfg[UPPER_MOTOR_ARM_J4310].node_id == 3U);
    assert(upper_motor_cfg[UPPER_MOTOR_CONVEYOR_M2006].can_bus == 3U);
    assert(upper_motor_cfg[UPPER_MOTOR_GRIPPER_M2006].can_bus == 3U);
}

static void Test_CheckFirstCycle(void)
{
    assert(test_tx_count == 3U);
    assert(test_tx[0].can_bus == 1U);
    assert(test_tx[0].frame.id == 3U);
    assert(test_tx[0].frame.data[7] == 0xFCU);

    assert(test_tx[1].can_bus == 2U);
    assert(test_tx[1].frame.id == 0x200U);
    assert(Test_ReadI16Be(&test_tx[1].frame.data[0]) > 0);
    assert(Test_ReadI16Be(&test_tx[1].frame.data[2]) > 0);
    assert(Test_ReadI16Be(&test_tx[1].frame.data[4]) == 0);
    assert(Test_ReadI16Be(&test_tx[1].frame.data[6]) == 0);

    assert(test_tx[2].can_bus == 3U);
    assert(test_tx[2].frame.id == 0x200U);
    assert(Test_ReadI16Be(&test_tx[2].frame.data[0]) > 0);
    assert(Test_ReadI16Be(&test_tx[2].frame.data[2]) < 0);
    assert(Test_ReadI16Be(&test_tx[2].frame.data[4]) == 0);
    assert(Test_ReadI16Be(&test_tx[2].frame.data[6]) == 0);
}

int main(void)
{
    Test_CheckTopology();
    assert(UpperMotorPort_Init(upper_motor_cfg, UPPER_MOTOR_COUNT));

    Test_FeedDji(2U, 1U, 0U);
    Test_FeedDji(2U, 2U, 0U);
    Test_FeedDji(3U, 1U, 0U);
    Test_FeedDji(3U, 2U, 0U);

    Test_ResetTx();
    Test_SendAll(1U);
    Test_CheckFirstCycle();

    Test_FeedJ4310(1U);
    Test_ResetTx();
    Test_SendAll(2U);
    assert(test_tx_count == 3U);
    assert(test_tx[0].can_bus == 1U);
    assert(test_tx[0].frame.id == 3U);
    assert(test_tx[0].frame.data[7] != 0xFCU);
    assert(test_tx[1].can_bus == 2U);
    assert(test_tx[1].frame.id == 0x200U);
    return 0;
}
