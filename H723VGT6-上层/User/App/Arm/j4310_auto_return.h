/**
 * @file j4310_auto_return.h
 * @brief 定义 J4310 自动回零控制器的状态和接口。
 */

#ifndef J4310_AUTO_RETURN_H
/** 防止 j4310_auto_return.h 被重复包含。 */
#define J4310_AUTO_RETURN_H

#include <stdbool.h>
#include <stdint.h>

/** 标识 J4310 自动回位流程当前执行到的阶段。 */
typedef enum
{
    J4310_AUTO_RETURN_DISABLED = 0, /**< 自动回位功能已关闭，不接管 J4310。 */
    J4310_AUTO_RETURN_ARMED, /**< 已检测到掉线，等待 J4310 重新上线。 */
    J4310_AUTO_RETURN_RUNNING, /**< 当前正在执行该流程。 */
    J4310_AUTO_RETURN_HOLDING /**< 已回到目标位置并保持当前关节角。 */
} j4310_auto_return_stage_t;

/** 保存 J4310 运行过程中需要集中管理的数据。 */
typedef struct
{
    bool enabled; /**< 对应控制功能是否启用。 */
    bool seen_online; /**< 自动回位控制器是否曾观察到 J4310 在线。 */
    bool online; /**< 对应设备当前是否在线。 */
    bool reconnect_armed; /**< J4310 重新上线时是否允许触发自动回位。 */
    bool owns_control; /**< 自动回位流程当前是否占用 J4310 控制权。 */
    j4310_auto_return_stage_t stage; /**< 自动回位或动作状态机当前执行的阶段。 */
    uint32_t trajectory_start_ms; /**< 当前轨迹开始执行的系统毫秒时刻。 */
    uint32_t trajectory_duration_ms; /**< 当前轨迹计划执行的总时间，单位：毫秒。 */
    float trajectory_start_position_rad; /**< J4310的轨迹起始位置，单位：弧度。 */
    float target_position_rad; /**< J4310的目标位置，单位：弧度。 */
    float target_velocity_rad_s; /**< J4310的目标速度，单位：弧度每秒。 */
} j4310_auto_return_t;

/* 功能：初始化 J4310 自动回零控制器；用途：设置使能状态并清空历史状态；无返回值表示控制器已复位。 */
void J4310AutoReturn_Init(j4310_auto_return_t *control /* 需要读取或更新的控制状态 */,
                          bool enabled /* 是否启用对应功能 */);
/* 功能：重新配置自动回零使能和反馈在线状态；用途：在系统启动或配置切换时重建状态机；无返回值表示配置已生效。 */
void J4310AutoReturn_Configure(j4310_auto_return_t *control /* 需要读取或更新的控制状态 */,
                               bool enabled /* 是否启用对应功能 */,
                               bool feedback_fresh /* 当前反馈是否仍在允许的超时时间内 */);
/* 功能：取消正在进行或等待中的自动回零；用途：在人工控制或停机时释放控制权；无返回值表示回零目标已清除。 */
void J4310AutoReturn_Cancel(j4310_auto_return_t *control /* 需要读取或更新的控制状态 */);
/* 功能：按反馈在线状态推进自动回零状态机；用途：检测掉线重连并生成回零目标；无返回值表示本周期状态已更新。 */
void J4310AutoReturn_Update(j4310_auto_return_t *control /* 需要读取或更新的控制状态 */,
                            uint32_t tick_ms /* 当前系统毫秒时刻 */,
                            bool feedback_fresh /* 当前反馈是否仍在允许的超时时间内 */,
                            float position_rad /* 目标或反馈位置，单位：弧度 */,
                            float velocity_rad_s /* 目标或反馈速度，单位：弧度每秒 */,
                            bool control_allowed /* 需要读取或更新的控制状态 */);
/* 功能：判断自动回零控制器是否持有控制权；用途：决定上层是否采用回零目标；返回 true 表示自动回零正在生效。 */
bool J4310AutoReturn_IsActive(const j4310_auto_return_t *control /* 需要读取或更新的控制状态 */);

#endif
