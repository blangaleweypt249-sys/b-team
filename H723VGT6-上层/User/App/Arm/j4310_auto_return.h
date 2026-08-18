/**
 * @file j4310_auto_return.h
 * @brief 定义 J4310 自动回零控制器的状态和接口。
 */

#ifndef J4310_AUTO_RETURN_H
#define J4310_AUTO_RETURN_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    J4310_AUTO_RETURN_DISABLED = 0,
    J4310_AUTO_RETURN_ARMED,
    J4310_AUTO_RETURN_RUNNING,
    J4310_AUTO_RETURN_HOLDING
} j4310_auto_return_stage_t;

typedef struct
{
    bool enabled;
    bool seen_online;
    bool online;
    bool reconnect_armed;
    bool owns_control;
    j4310_auto_return_stage_t stage;
    uint32_t trajectory_start_ms;
    uint32_t trajectory_duration_ms;
    float trajectory_start_position_rad;
    float target_position_rad;
    float target_velocity_rad_s;
} j4310_auto_return_t;

/* 功能：初始化 J4310 自动回零控制器；用途：设置使能状态并清空历史状态；无返回值表示控制器已复位。 */
void J4310AutoReturn_Init(j4310_auto_return_t *control,
                          bool enabled);
/* 功能：重新配置自动回零使能和反馈在线状态；用途：在系统启动或配置切换时重建状态机；无返回值表示配置已生效。 */
void J4310AutoReturn_Configure(j4310_auto_return_t *control,
                               bool enabled,
                               bool feedback_fresh);
/* 功能：取消正在进行或等待中的自动回零；用途：在人工控制或停机时释放控制权；无返回值表示回零目标已清除。 */
void J4310AutoReturn_Cancel(j4310_auto_return_t *control);
/* 功能：按反馈在线状态推进自动回零状态机；用途：检测掉线重连并生成回零目标；无返回值表示本周期状态已更新。 */
void J4310AutoReturn_Update(j4310_auto_return_t *control,
                            uint32_t tick_ms,
                            bool feedback_fresh,
                            float position_rad,
                            float velocity_rad_s,
                            bool control_allowed);
/* 功能：判断自动回零控制器是否持有控制权；用途：决定上层是否采用回零目标；返回 true 表示自动回零正在生效。 */
bool J4310AutoReturn_IsActive(const j4310_auto_return_t *control);

#endif
