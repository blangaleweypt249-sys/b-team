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

/** 该通信帧完整编码后的固定字节数。 */
#define TEST_FRAME_SIZE 10U

/* 功能：构造一帧固定格式的遥控测试数据；用途：生成不同按键、摇杆和序列号输入；返回值表示完整帧数据。 */
static void Test_BuildFixedFrame(uint8_t frame[TEST_FRAME_SIZE] /* 需要解析或发送的 CAN 或协议帧 */,
                                 uint8_t left_x /* 主遥控左摇杆横向原始值 */,
                                 uint8_t left_y /* 主遥控左摇杆纵向原始值 */,
                                 uint8_t right_x /* 主遥控右摇杆横向原始值 */,
                                 uint8_t right_y /* 主遥控右摇杆纵向原始值 */,
                                 uint8_t primary_keys /* 主遥控 PC0、PC1 按键的当前位图 */,
                                 uint8_t primary_switch /* 主遥控 PE0、PD6 开关的当前位图 */,
                                 uint8_t secondary_keys /* 副遥控 PD8 至 PD13 按键的当前位图 */,
                                 uint8_t secondary_switches /* 副遥控开关的当前位图 */)
{
    frame[0] = 0xA5U;
    frame[1] = 0x5AU;
    frame[2] = left_x;
    frame[3] = left_y;
    frame[4] = right_x;
    frame[5] = right_y;
    frame[6] = primary_keys;
    frame[7] = primary_switch;
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
                         0U, 0xFFU, 0x15U, 0x21U);
    UpperRemoteLink_Init(&link);
    UpperRemoteLink_Push(&link, frame, 3U, 10U);
    assert(!UpperRemoteLink_GetControl(&link, 10U, &control));
    UpperRemoteLink_Push(&link, &frame[3], sizeof(frame) - 3U, 11U);
    assert(UpperRemoteLink_GetControl(&link, 11U, &control));
    assert(control.online);
    assert(control.primary_key_bits == 0U);
    assert(control.primary_switch == UPPER_REMOTE_PRIMARY_SWITCH_MASK);
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
                         0U, UPPER_REMOTE_PRIMARY_SWITCH_MASK, 0U, 0U);
    Test_BuildFixedFrame(&stream[TEST_FRAME_SIZE], 128U, 128U, 128U, 128U,
                         0xFFU, 0xFFU, 0xFFU, 0xFFU);
    UpperRemoteLink_Init(&link);
    UpperRemoteLink_Push(&link, stream, sizeof(stream), 25U);
    assert(UpperRemoteLink_GetControl(&link, 25U, &control));
    assert(control.primary_key_bits ==
           (UPPER_REMOTE_PRIMARY_KEY_PC0 | UPPER_REMOTE_PRIMARY_KEY_PC1));
    assert(control.primary_switch == UPPER_REMOTE_PRIMARY_SWITCH_MASK);
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
                         UPPER_REMOTE_PRIMARY_KEY_PC0,
                         UPPER_REMOTE_PRIMARY_SWITCH_PD6,
                         0x01U, 0x10U);
    UpperRemoteLink_Init(&link);
    UpperRemoteLink_Push(&link, prefix, sizeof(prefix), 30U);
    UpperRemoteLink_Push(&link, frame, sizeof(frame), 31U);
    assert(UpperRemoteLink_GetControl(&link, 31U, &control));
    assert(control.primary_key_bits == UPPER_REMOTE_PRIMARY_KEY_PC0);
    assert(control.primary_switch == UPPER_REMOTE_PRIMARY_SWITCH_PD6);
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
    assert(control.primary_key_bits ==
           (UPPER_REMOTE_PRIMARY_KEY_PC0 | UPPER_REMOTE_PRIMARY_KEY_PC1));
    assert(control.key_bits == 0x05U);
    assert(control.switch_bits == 0x06U);
    assert(link.diagnostics.valid_frame_count == 2U);
}

/* 普通按键和普通开关只能更新各自字段，不能改变 PE0/PD6 模式。 */
static void Test_OrdinaryInputsDoNotChangeMode(void)
{
    uint8_t frame[TEST_FRAME_SIZE];
    upper_remote_link_t link;
    upper_remote_control_t control;
    uint8_t index;

    UpperRemoteLink_Init(&link);
    Test_BuildFixedFrame(frame, 128U, 128U, 128U, 128U,
                         0U, UPPER_REMOTE_PRIMARY_SWITCH_PE0, 0U, 0U);
    UpperRemoteLink_Push(&link, frame, sizeof(frame), 60U);

    for (index = 0U; index < 6U; index++)
    {
        Test_BuildFixedFrame(
            frame, 128U, 128U, 128U, 128U,
            (index < 2U) ? (uint8_t)(1U << (index + 1U)) : 0U,
            UPPER_REMOTE_PRIMARY_SWITCH_PE0,
            (uint8_t)(1U << index),
            (uint8_t)((1U << index) & UPPER_REMOTE_SWITCH_MASK));
        UpperRemoteLink_Push(&link, frame, sizeof(frame),
                             (uint32_t)(61U + index));
        assert(UpperRemoteLink_GetControl(&link,
                                          (uint32_t)(61U + index),
                                          &control));
        assert(control.primary_switch ==
               UPPER_REMOTE_PRIMARY_SWITCH_PE0);
    }
}

/* PE0 与 PD6 各自需要连续稳定，单帧异常不会改变另一维模式。 */
static void Test_ModeSwitchesAreIndependentAndFiltered(void)
{
    uint8_t frame[TEST_FRAME_SIZE];
    upper_remote_link_t link;
    upper_remote_control_t control;
    uint8_t index;
    uint32_t tick_ms = 72U;

    UpperRemoteLink_Init(&link);
    Test_BuildFixedFrame(frame, 128U, 128U, 128U, 128U,
                         0U, UPPER_REMOTE_PRIMARY_SWITCH_PE0, 0U, 0U);
    UpperRemoteLink_Push(&link, frame, sizeof(frame), 70U);

    /* 一帧 PE0 丢失模拟干扰，仍保持自动存二。 */
    frame[7] = 0U;
    UpperRemoteLink_Push(&link, frame, sizeof(frame), 71U);
    assert(UpperRemoteLink_GetControl(&link, 71U, &control));
    assert(control.primary_switch == UPPER_REMOTE_PRIMARY_SWITCH_PE0);

    /* PE0 恢复，同时只切 PD6；确认后成为自动存三。 */
    frame[7] = UPPER_REMOTE_PRIMARY_SWITCH_PE0 |
               UPPER_REMOTE_PRIMARY_SWITCH_PD6;
    for (index = 0U; index < UPPER_REMOTE_MODE_STABLE_FRAMES; index++)
    {
        UpperRemoteLink_Push(&link, frame, sizeof(frame), tick_ms++);
    }
    assert(UpperRemoteLink_GetControl(&link, tick_ms, &control));
    assert(control.primary_switch ==
           (UPPER_REMOTE_PRIMARY_SWITCH_PE0 |
            UPPER_REMOTE_PRIMARY_SWITCH_PD6));

    /* 只释放 PE0：自动存三 -> 手动存三，PD6 保持不变。 */
    frame[7] = UPPER_REMOTE_PRIMARY_SWITCH_PD6;
    for (index = 0U; index < UPPER_REMOTE_MODE_STABLE_FRAMES; index++)
    {
        UpperRemoteLink_Push(&link, frame, sizeof(frame), tick_ms++);
    }
    assert(UpperRemoteLink_GetControl(&link, tick_ms, &control));
    assert(control.primary_switch == UPPER_REMOTE_PRIMARY_SWITCH_PD6);

    /* 再释放 PD6：手动存三 -> 手动存二。 */
    frame[7] = 0U;
    for (index = 0U; index < UPPER_REMOTE_MODE_STABLE_FRAMES; index++)
    {
        UpperRemoteLink_Push(&link, frame, sizeof(frame), tick_ms++);
    }
    assert(UpperRemoteLink_GetControl(&link, tick_ms, &control));
    assert(control.primary_switch == 0U);

    /* 只闭合 PE0：手动存二 -> 自动存二，PD6 保持不变。 */
    frame[7] = UPPER_REMOTE_PRIMARY_SWITCH_PE0;
    for (index = 0U; index < UPPER_REMOTE_MODE_STABLE_FRAMES; index++)
    {
        UpperRemoteLink_Push(&link, frame, sizeof(frame), tick_ms++);
    }
    assert(UpperRemoteLink_GetControl(&link, tick_ms, &control));
    assert(control.primary_switch == UPPER_REMOTE_PRIMARY_SWITCH_PE0);
}

/* 功能：运行本文件的遥控链路的分帧、粘包和异常数据重同步测试；用途：集中执行断言用例；返回 0 表示全部测试通过。 */
int main(void)
{
    Test_SplitFrame();
    Test_ConcatenatedFramesAndMasks();
    Test_NoiseAndHeaderResynchronization();
    Test_ExtremeJoystickValuesAreStillFixedFrames();
    Test_PayloadMayContainHeaderBytes();
    Test_OrdinaryInputsDoNotChangeMode();
    Test_ModeSwitchesAreIndependentAndFiltered();
    return 0;
}
