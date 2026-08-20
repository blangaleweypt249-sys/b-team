/**
 * @file test_upper_remote_link.c
 * @brief 验证遥控链路的分帧、粘包和异常数据重同步。
 */

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "upper_remote_link.h"

#define TEST_FRAME_SIZE 10U

/* 功能：构造一帧固定格式的遥控测试数据；用途：生成不同按键、摇杆和序列号输入；返回值表示完整帧数据。 */
static void Test_BuildFixedFrame(uint8_t frame[TEST_FRAME_SIZE],
                                 uint8_t left_x,
                                 uint8_t left_y,
                                 uint8_t right_x,
                                 uint8_t right_y,
                                 uint8_t reserved_6,
                                 uint8_t reserved_7,
                                 uint8_t secondary_keys,
                                 uint8_t secondary_switches)
{
    frame[0] = 0xA5U;
    frame[1] = 0x5AU;
    frame[2] = left_x;
    frame[3] = left_y;
    frame[4] = right_x;
    frame[5] = right_y;
    frame[6] = reserved_6;
    frame[7] = reserved_7;
    frame[8] = secondary_keys;
    frame[9] = secondary_switches;
}

/* 功能：执行 SplitFrame 场景测试；用途：验证对应输入下的行为和断言；无返回值表示由断言报告结果。 */
static void Test_SplitFrame(void)
{
    uint8_t frame[TEST_FRAME_SIZE];
    upper_remote_link_t link;
    upper_remote_control_t control;

    Test_BuildFixedFrame(frame, 128U, 128U, 128U, 128U,
                         0U, 0U, 0x15U, 0x21U);
    UpperRemoteLink_Init(&link);
    UpperRemoteLink_Push(&link, frame, 3U, 10U);
    assert(!UpperRemoteLink_GetControl(&link, 10U, &control));
    UpperRemoteLink_Push(&link, &frame[3], sizeof(frame) - 3U, 11U);
    assert(UpperRemoteLink_GetControl(&link, 11U, &control));
    assert(control.online);
    assert(control.key_bits == 0x15U);
    assert(control.switch_bits == (0x21U & UPPER_REMOTE_SWITCH_MASK));
    assert(control.sequence == 0U);
    assert(control.updated_at_ms == 11U);
    assert(link.diagnostics.valid_frame_count == 1U);
}

/* 功能：执行 ConcatenatedFramesAndMasks 场景测试；用途：验证对应输入下的行为和断言；无返回值表示由断言报告结果。 */
static void Test_ConcatenatedFramesAndMasks(void)
{
    uint8_t stream[TEST_FRAME_SIZE * 2U];
    upper_remote_link_t link;
    upper_remote_control_t control;

    Test_BuildFixedFrame(&stream[0], 128U, 128U, 128U, 128U,
                         0U, 0U, 0U, 0U);
    Test_BuildFixedFrame(&stream[TEST_FRAME_SIZE], 128U, 128U, 128U, 128U,
                         0U, 0U, 0xFFU, 0xFFU);
    UpperRemoteLink_Init(&link);
    UpperRemoteLink_Push(&link, stream, sizeof(stream), 25U);
    assert(UpperRemoteLink_GetControl(&link, 25U, &control));
    assert(control.key_bits == UPPER_REMOTE_KEY_MASK);
    assert(control.switch_bits == UPPER_REMOTE_SWITCH_MASK);
    assert(link.diagnostics.valid_frame_count == 2U);
}

/* 功能：执行 NoiseAndHeaderResynchronization 场景测试；用途：验证对应输入下的行为和断言；无返回值表示由断言报告结果。 */
static void Test_NoiseAndHeaderResynchronization(void)
{
    const uint8_t prefix[] = {0x01U, 0x02U, 0xA5U, 0x10U};
    uint8_t frame[TEST_FRAME_SIZE];
    upper_remote_link_t link;
    upper_remote_control_t control;

    Test_BuildFixedFrame(frame, 1U, 2U, 6U, 9U,
                         0U, 0U, 0x01U, 0x10U);
    UpperRemoteLink_Init(&link);
    UpperRemoteLink_Push(&link, prefix, sizeof(prefix), 30U);
    UpperRemoteLink_Push(&link, frame, sizeof(frame), 31U);
    assert(UpperRemoteLink_GetControl(&link, 31U, &control));
    assert(control.key_bits == 0x01U);
    assert(control.switch_bits == 0x10U);
    assert(link.diagnostics.valid_frame_count == 1U);
}

/* 功能：执行 ExtremeJoystickValuesAreStillFixedFrames 场景测试；用途：验证对应输入下的行为和断言；无返回值表示由断言报告结果。 */
static void Test_ExtremeJoystickValuesAreStillFixedFrames(void)
{
    uint8_t frame[TEST_FRAME_SIZE];
    upper_remote_link_t link;
    upper_remote_control_t control;

    /* 这些数值过去曾类似于已移除的 CRC 帧签名。 */
    Test_BuildFixedFrame(frame, 2U, 1U, 2U, 6U,
                         0U, 0U, 0x2AU, 0x15U);
    UpperRemoteLink_Init(&link);
    UpperRemoteLink_Push(&link, frame, sizeof(frame), 40U);
    assert(UpperRemoteLink_GetControl(&link, 40U, &control));
    assert(control.key_bits == 0x2AU);
    assert(control.switch_bits == (0x15U & UPPER_REMOTE_SWITCH_MASK));
    assert(link.diagnostics.valid_frame_count == 1U);
}

/* 功能：执行 PayloadMayContainHeaderBytes 场景测试；用途：验证对应输入下的行为和断言；无返回值表示由断言报告结果。 */
static void Test_PayloadMayContainHeaderBytes(void)
{
    uint8_t stream[TEST_FRAME_SIZE * 2U];
    upper_remote_link_t link;
    upper_remote_control_t control;

    Test_BuildFixedFrame(&stream[0], 0xA5U, 0x5AU, 0xA5U, 0x5AU,
                         0U, 0U, 0x03U, 0x04U);
    Test_BuildFixedFrame(&stream[TEST_FRAME_SIZE], 128U, 128U, 128U, 128U,
                         0xC7U, 0U, 0x05U, 0x06U);
    UpperRemoteLink_Init(&link);
    UpperRemoteLink_Push(&link, stream, sizeof(stream), 50U);
    assert(UpperRemoteLink_GetControl(&link, 50U, &control));
    assert(control.key_bits == 0x05U);
    assert(control.switch_bits == 0x06U);
    assert(link.diagnostics.valid_frame_count == 2U);
}

/* 功能：运行本文件的遥控链路的分帧、粘包和异常数据重同步测试；用途：集中执行断言用例；返回 0 表示全部测试通过。 */
int main(void)
{
    Test_SplitFrame();
    Test_ConcatenatedFramesAndMasks();
    Test_NoiseAndHeaderResynchronization();
    Test_ExtremeJoystickValuesAreStillFixedFrames();
    Test_PayloadMayContainHeaderBytes();
    return 0;
}
