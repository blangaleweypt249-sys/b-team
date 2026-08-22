/**
 * @file position_stall_monitor.h
 * @brief 检测位置控制执行机构是否持续堵转。
 */

#ifndef POSITION_STALL_MONITOR_H
#define POSITION_STALL_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float minimum_error_rad;
    float maximum_velocity_rad_s;
    float minimum_effort;
    uint32_t arming_grace_ms;
    uint32_t stall_duration_ms;
} position_stall_monitor_cfg_t;

typedef struct
{
    position_stall_monitor_cfg_t cfg;
    float target_position_rad;
    uint32_t armed_at_ms;
    uint32_t stall_started_at_ms;
    bool armed;
    bool stall_timing;
} position_stall_monitor_t;

/* 初始化监测器并设置保守的持续堵转阈值。 */
bool PositionStallMonitor_Init(
    position_stall_monitor_t *monitor,
    const position_stall_monitor_cfg_t *cfg);
/* 针对明确请求的位置目标启动或重新启动监测。 */
void PositionStallMonitor_Arm(position_stall_monitor_t *monitor,
                              float target_position_rad,
                              uint32_t tick_ms);
/* 停止监测并丢弃尚未完成的堵转计时。 */
void PositionStallMonitor_Disarm(position_stall_monitor_t *monitor);
/* 当所有堵转条件持续满足指定时长时，仅返回一次 true。 */
bool PositionStallMonitor_Update(position_stall_monitor_t *monitor,
                                 uint32_t tick_ms,
                                 bool feedback_valid,
                                 float position_rad,
                                 float velocity_rad_s,
                                 float effort);

#endif
