/**
 * @file test_position_stall_monitor.c
 * @brief 验证保守的持续堵转计时和重新启动逻辑。
 */

#include <assert.h>

#include "position_stall_monitor.h"

int main(void)
{
    position_stall_monitor_cfg_t cfg =
    {
        .minimum_error_rad = 0.20f,
        .maximum_velocity_rad_s = 0.02f,
        .minimum_effort = 3.0f,
        .arming_grace_ms = 500U,
        .stall_duration_ms = 3000U
    };
    position_stall_monitor_t monitor;

    assert(PositionStallMonitor_Init(&monitor, &cfg));
    PositionStallMonitor_Arm(&monitor, 1.0f, 100U);
    assert(!PositionStallMonitor_Update(
        &monitor, 599U, true, 0.0f, 0.0f, 4.0f));
    assert(!PositionStallMonitor_Update(
        &monitor, 600U, true, 0.0f, 0.0f, 4.0f));
    assert(!PositionStallMonitor_Update(
        &monitor, 3599U, true, 0.0f, 0.0f, 4.0f));
    assert(PositionStallMonitor_Update(
        &monitor, 3600U, true, 0.0f, 0.0f, 4.0f));
    assert(!PositionStallMonitor_Update(
        &monitor, 7000U, true, 0.0f, 0.0f, 4.0f));

    /* 发生运动、输出力度过低、误差过小或反馈过期时，连续状态中断。 */
    PositionStallMonitor_Arm(&monitor, 1.0f, 8000U);
    assert(!PositionStallMonitor_Update(
        &monitor, 8500U, true, 0.0f, 0.0f, 4.0f));
    assert(!PositionStallMonitor_Update(
        &monitor, 9000U, true, 0.0f, 0.03f, 4.0f));
    assert(!PositionStallMonitor_Update(
        &monitor, 9500U, true, 0.0f, 0.0f, 2.0f));
    assert(!PositionStallMonitor_Update(
        &monitor, 10000U, true, 0.85f, 0.0f, 4.0f));
    assert(!PositionStallMonitor_Update(
        &monitor, 10500U, false, 0.0f, 0.0f, 4.0f));
    assert(!PositionStallMonitor_Update(
        &monitor, 11000U, true, 0.0f, 0.0f, 4.0f));
    assert(!PositionStallMonitor_Update(
        &monitor, 13999U, true, 0.0f, 0.0f, 4.0f));
    assert(PositionStallMonitor_Update(
        &monitor, 14000U, true, 0.0f, 0.0f, 4.0f));

    /* 重复提交相同目标不得推迟堵转检测。 */
    PositionStallMonitor_Arm(&monitor, 2.0f, 15000U);
    assert(!PositionStallMonitor_Update(
        &monitor, 15500U, true, 0.0f, 0.0f, 4.0f));
    PositionStallMonitor_Arm(&monitor, 2.0f, 16000U);
    assert(PositionStallMonitor_Update(
        &monitor, 18500U, true, 0.0f, 0.0f, 4.0f));

    return 0;
}
