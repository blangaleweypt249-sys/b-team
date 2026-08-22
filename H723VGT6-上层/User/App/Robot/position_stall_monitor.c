/**
 * @file position_stall_monitor.c
 * @brief 实现连续位置堵转检测。
 */

#include "position_stall_monitor.h"

#include <math.h>
#include <string.h>

bool PositionStallMonitor_Init(
    position_stall_monitor_t *monitor,
    const position_stall_monitor_cfg_t *cfg)
{
    if ((monitor == NULL) || (cfg == NULL) ||
        !isfinite(cfg->minimum_error_rad) ||
        !isfinite(cfg->maximum_velocity_rad_s) ||
        !isfinite(cfg->minimum_effort) ||
        (cfg->minimum_error_rad <= 0.0f) ||
        (cfg->maximum_velocity_rad_s < 0.0f) ||
        (cfg->minimum_effort < 0.0f) ||
        (cfg->stall_duration_ms == 0U))
    {
        return false;
    }

    (void)memset(monitor, 0, sizeof(*monitor));
    monitor->cfg = *cfg;
    return true;
}

void PositionStallMonitor_Arm(position_stall_monitor_t *monitor,
                              float target_position_rad,
                              uint32_t tick_ms)
{
    if ((monitor == NULL) || !isfinite(target_position_rad))
    {
        return;
    }
    if (monitor->armed &&
        (monitor->target_position_rad == target_position_rad))
    {
        return;
    }

    monitor->target_position_rad = target_position_rad;
    monitor->armed_at_ms = tick_ms;
    monitor->stall_started_at_ms = 0U;
    monitor->armed = true;
    monitor->stall_timing = false;
}

void PositionStallMonitor_Disarm(position_stall_monitor_t *monitor)
{
    if (monitor == NULL)
    {
        return;
    }

    monitor->armed = false;
    monitor->stall_timing = false;
    monitor->stall_started_at_ms = 0U;
}

bool PositionStallMonitor_Update(position_stall_monitor_t *monitor,
                                 uint32_t tick_ms,
                                 bool feedback_valid,
                                 float position_rad,
                                 float velocity_rad_s,
                                 float effort)
{
    bool stalled;

    if ((monitor == NULL) || !monitor->armed)
    {
        return false;
    }
    if (!feedback_valid || !isfinite(position_rad) ||
        !isfinite(velocity_rad_s) || !isfinite(effort) ||
        ((tick_ms - monitor->armed_at_ms) < monitor->cfg.arming_grace_ms))
    {
        monitor->stall_timing = false;
        return false;
    }

    stalled =
        (fabsf(monitor->target_position_rad - position_rad) >=
         monitor->cfg.minimum_error_rad) &&
        (fabsf(velocity_rad_s) <= monitor->cfg.maximum_velocity_rad_s) &&
        (fabsf(effort) >= monitor->cfg.minimum_effort);
    if (!stalled)
    {
        monitor->stall_timing = false;
        return false;
    }
    if (!monitor->stall_timing)
    {
        monitor->stall_timing = true;
        monitor->stall_started_at_ms = tick_ms;
        return false;
    }
    if ((tick_ms - monitor->stall_started_at_ms) <
        monitor->cfg.stall_duration_ms)
    {
        return false;
    }

    PositionStallMonitor_Disarm(monitor);
    return true;
}
