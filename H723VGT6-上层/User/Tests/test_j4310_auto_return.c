/**
 * @file test_j4310_auto_return.c
 * @brief 验证 J4310 重连自动回零状态机和轨迹行为。
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "j4310_auto_return.h"

/* 功能：判断两个浮点数是否在给定误差内接近；用途：辅助验证数值控制结果；返回 true 表示比较通过。 */
static bool Test_Close(float actual, float expected, float tolerance)
{
    float error = actual - expected;

    return (error >= -tolerance) && (error <= tolerance);
}

/* 功能：运行本文件的 J4310 重连自动回零状态机和轨迹行为测试；用途：集中执行断言用例；返回 0 表示全部测试通过。 */
int main(void)
{
    j4310_auto_return_t control;
    float positive_midpoint;

    /* MCU 启动时，第一帧反馈只用于建立在线基线。 */
    J4310AutoReturn_Init(&control, false);
    J4310AutoReturn_Update(&control, 10U, true, 1.0f, 0.0f, true);
    assert(control.seen_online);
    assert(!control.owns_control);
    assert(control.stage == J4310_AUTO_RETURN_DISABLED);

    /* 电机在线时执行使能不会移动电机。 */
    J4310AutoReturn_Configure(&control, true, true);
    assert(!control.reconnect_armed);
    J4310AutoReturn_Update(&control, 20U, true, 2.0f, 0.0f, true);
    assert(!control.owns_control);

    /* 只有电机反馈丢失后重新连接，才会启动回零。 */
    J4310AutoReturn_Update(&control, 30U, false, 0.0f, 0.0f, true);
    assert(control.reconnect_armed);
    J4310AutoReturn_Update(&control, 40U, true, 2.0f, 0.0f, true);
    assert(control.owns_control);
    assert(control.stage == J4310_AUTO_RETURN_RUNNING);
    assert(Test_Close(control.target_position_rad, 2.0f, 0.0001f));
    J4310AutoReturn_Update(&control, 1040U, true, 1.0f, -0.5f, true);
    positive_midpoint = control.target_position_rad;
    assert(positive_midpoint > 0.0f && positive_midpoint < 2.0f);
    assert(control.target_velocity_rad_s < 0.0f);

    J4310AutoReturn_Update(&control, 2040U, false, 0.0f, 0.0f, true);
    assert(!control.owns_control);
    assert(control.reconnect_armed);
    J4310AutoReturn_Update(&control, 2050U, true, -1.5f, 0.0f, true);
    assert(control.owns_control);
    assert(Test_Close(control.target_position_rad, -1.5f, 0.0001f));
    J4310AutoReturn_Update(&control, 3050U, true, -0.7f, 0.5f, true);
    assert(control.target_position_rad > -1.5f);
    assert(control.target_position_rad < 0.0f);
    assert(control.target_velocity_rad_s > 0.0f);

    J4310AutoReturn_Update(&control, 5000U, true, 0.01f, 0.05f, true);
    assert(control.owns_control);
    assert(control.stage == J4310_AUTO_RETURN_HOLDING);
    assert(control.target_position_rad == 0.0f);
    assert(control.target_velocity_rad_s == 0.0f);

    J4310AutoReturn_Cancel(&control);
    assert(!control.owns_control);
    assert(!control.reconnect_armed);
    J4310AutoReturn_Update(&control, 5010U, true, 0.5f, 0.0f, true);
    assert(!control.owns_control);
    J4310AutoReturn_Update(&control, 5020U, false, 0.0f, 0.0f, true);
    J4310AutoReturn_Update(&control, 5030U, true, 0.5f, 0.0f, true);
    assert(control.owns_control);
    assert(control.trajectory_duration_ms <= 550U);

    J4310AutoReturn_Configure(&control, true, true);
    assert(!control.owns_control);
    assert(!control.reconnect_armed);
    J4310AutoReturn_Update(&control, 6000U, true, 0.8f, 0.0f, true);
    assert(!control.owns_control);
    J4310AutoReturn_Update(&control, 6010U, false, 0.0f, 0.0f, true);
    J4310AutoReturn_Update(&control, 6020U, true, 0.8f, 0.0f, true);
    assert(control.owns_control);

    J4310AutoReturn_Update(&control, 6030U, true, 0.7f, 0.0f, false);
    assert(!control.owns_control);
    assert(control.stage == J4310_AUTO_RETURN_ARMED);

    /* 仅电机重新上电时，急停状态不得准备或启动回零。 */
    J4310AutoReturn_Cancel(&control);
    J4310AutoReturn_Update(&control, 6040U, false, 0.0f, 0.0f, false);
    assert(!control.reconnect_armed);
    J4310AutoReturn_Update(&control, 6050U, true, 0.7f, 0.0f, false);
    assert(!control.owns_control);

    J4310AutoReturn_Configure(&control, false, true);
    assert(!control.enabled);
    assert(control.stage == J4310_AUTO_RETURN_DISABLED);

    /* 新的 MCU 会话默认关闭；即使轴偏离零点，也不能把第一帧反馈视为仅电机重启。 */
    J4310AutoReturn_Init(&control, false);
    assert(!control.enabled);
    J4310AutoReturn_Update(&control, 7000U, true, 2.5f, 0.0f, true);
    assert(!control.owns_control);
    assert(!control.reconnect_armed);

    /* MCU 从未见过该电机时提前使能，也不能把首次连接误认为重新连接。 */
    J4310AutoReturn_Init(&control, true);
    assert(!control.reconnect_armed);
    J4310AutoReturn_Update(&control, 8000U, true, -2.5f, 0.0f, true);
    assert(!control.owns_control);

    /* 如果电机在消失前曾被检测到，离线期间使能可以准备下一次真实重连。 */
    J4310AutoReturn_Init(&control, false);
    J4310AutoReturn_Update(&control, 9000U, true, 0.5f, 0.0f, true);
    J4310AutoReturn_Update(&control, 9010U, false, 0.0f, 0.0f, true);
    J4310AutoReturn_Configure(&control, true, false);
    assert(control.reconnect_armed);
    J4310AutoReturn_Update(&control, 9020U, true, 0.5f, 0.0f, true);
    assert(control.owns_control);
    return 0;
}
