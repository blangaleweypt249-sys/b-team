/**
 * @file position_stall_monitor.h
 * @brief 检测位置控制执行机构是否持续堵转。
 */

#ifndef POSITION_STALL_MONITOR_H
/** 防止 position_stall_monitor.h 被重复包含。 */
#define POSITION_STALL_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

/** 保存 模块 初始化和控制所需的配置参数。 */
typedef struct
{
    float minimum_error_rad; /**< 开始判断堵转所需的最小绝对位置误差，单位：弧度。 */
    float maximum_velocity_rad_s; /**< 模块的最大允许速度，单位：弧度每秒。 */
    float minimum_effort; /**< 开始判断堵转所需的最小转矩或电流绝对值。 */
    uint32_t arming_grace_ms; /**< 位置目标变化后暂不判断堵转的宽限时间，单位：毫秒。 */
    uint32_t stall_duration_ms; /**< 堵转条件必须连续成立的确认时间，单位：毫秒。 */
} position_stall_monitor_cfg_t;

/** 保存 模块 运行过程中需要集中管理的数据。 */
typedef struct
{
    position_stall_monitor_cfg_t cfg; /**< 模块当前使用的配置参数。 */
    float target_position_rad; /**< 模块的目标位置，单位：弧度。 */
    uint32_t armed_at_ms; /**< 本次堵转监测开始生效的系统毫秒时刻。 */
    uint32_t stall_started_at_ms; /**< 堵转条件本次开始连续成立的系统毫秒时刻。 */
    bool armed; /**< 堵转监测是否已经越过宽限期。 */
    bool stall_timing; /**< 堵转条件是否正在持续计时。 */
} position_stall_monitor_t;

/* 初始化监测器并设置保守的持续堵转阈值。 */
bool PositionStallMonitor_Init(
    position_stall_monitor_t *monitor /* 函数读取或写入的对象地址 */,
    const position_stall_monitor_cfg_t *cfg /* 初始化或更新时使用的配置参数 */);
/* 针对明确请求的位置目标启动或重新启动监测。 */
void PositionStallMonitor_Arm(position_stall_monitor_t *monitor /* 函数读取或写入的对象地址 */,
                              float target_position_rad /* 轨迹目标关节角，单位：弧度 */,
                              uint32_t tick_ms /* 当前系统毫秒时刻 */);
/* 停止监测并丢弃尚未完成的堵转计时。 */
void PositionStallMonitor_Disarm(position_stall_monitor_t *monitor /* 函数读取或写入的对象地址 */);
/* 当所有堵转条件持续满足指定时长时，仅返回一次 true。 */
bool PositionStallMonitor_Update(position_stall_monitor_t *monitor /* 函数读取或写入的对象地址 */,
                                 uint32_t tick_ms /* 当前系统毫秒时刻 */,
                                 bool feedback_valid /* 当前反馈快照是否有效 */,
                                 float position_rad /* 目标或反馈位置，单位：弧度 */,
                                 float velocity_rad_s /* 目标或反馈速度，单位：弧度每秒 */,
                                 float effort /* 当前控制输出的转矩或电流绝对值 */);

#endif
