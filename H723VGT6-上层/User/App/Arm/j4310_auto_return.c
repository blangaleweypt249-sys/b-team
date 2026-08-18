/**
 * @file j4310_auto_return.c
 * @brief 实现 J4310 电机重连后的自动回零轨迹控制。
 */

#include "j4310_auto_return.h"

#include <math.h>
#include <string.h>

#define J4310_AUTO_RETURN_CONTROL_PERIOD_MS       10U
#define J4310_AUTO_RETURN_MAX_VELOCITY_RAD_S       2.0f
#define J4310_AUTO_RETURN_MAX_ACCEL_RAD_S2        10.0f
#define J4310_AUTO_RETURN_SETTLE_POSITION_RAD      0.03f
#define J4310_AUTO_RETURN_SETTLE_VELOCITY_RAD_S    0.20f
#define J4310_AUTO_RETURN_QUINTIC_VELOCITY_BOUND   1.875f
#define J4310_AUTO_RETURN_QUINTIC_ACCEL_BOUND      5.7736f

/* 功能：按回零距离计算五次轨迹持续时间；用途：同时满足最大速度和加速度约束；返回值表示轨迹时长（毫秒）。 */
static uint32_t J4310AutoReturn_TrajectoryDurationMs(float distance_rad)
{
    float velocity_time_s;
    float acceleration_time_s;
    float duration_ms;
    uint32_t period_count;

    distance_rad = fabsf(distance_rad);
    if (distance_rad <= J4310_AUTO_RETURN_SETTLE_POSITION_RAD)
    {
        return 0U;
    }
    velocity_time_s = J4310_AUTO_RETURN_QUINTIC_VELOCITY_BOUND *
                      distance_rad /
                      J4310_AUTO_RETURN_MAX_VELOCITY_RAD_S;
    acceleration_time_s = sqrtf(
        J4310_AUTO_RETURN_QUINTIC_ACCEL_BOUND * distance_rad /
        J4310_AUTO_RETURN_MAX_ACCEL_RAD_S2);
    duration_ms = ((velocity_time_s > acceleration_time_s) ?
                   velocity_time_s : acceleration_time_s) * 1000.0f;
    period_count = (uint32_t)(duration_ms /
                              (float)J4310_AUTO_RETURN_CONTROL_PERIOD_MS);
    if ((float)(period_count * J4310_AUTO_RETURN_CONTROL_PERIOD_MS) <
        duration_ms)
    {
        period_count++;
    }
    if (period_count == 0U)
    {
        period_count = 1U;
    }
    return period_count * J4310_AUTO_RETURN_CONTROL_PERIOD_MS;
}

/* 功能：采样当前自动回零五次轨迹；用途：生成平滑的位置和速度目标；无返回值表示目标已写入控制器。 */
static void J4310AutoReturn_Sample(j4310_auto_return_t *control,
                                   uint32_t tick_ms)
{
    uint32_t elapsed_ms;
    float normalized_time;
    float normalized_time_2;
    float normalized_time_3;
    float normalized_time_4;
    float normalized_time_5;
    float blend;
    float blend_rate;

    if (control->trajectory_duration_ms == 0U)
    {
        control->target_position_rad = 0.0f;
        control->target_velocity_rad_s = 0.0f;
        return;
    }
    elapsed_ms = tick_ms - control->trajectory_start_ms;
    if (elapsed_ms >= control->trajectory_duration_ms)
    {
        control->target_position_rad = 0.0f;
        control->target_velocity_rad_s = 0.0f;
        return;
    }

    normalized_time = (float)elapsed_ms /
                      (float)control->trajectory_duration_ms;
    normalized_time_2 = normalized_time * normalized_time;
    normalized_time_3 = normalized_time_2 * normalized_time;
    normalized_time_4 = normalized_time_3 * normalized_time;
    normalized_time_5 = normalized_time_4 * normalized_time;
    blend = 10.0f * normalized_time_3 -
            15.0f * normalized_time_4 +
            6.0f * normalized_time_5;
    blend_rate = (30.0f * normalized_time_2 -
                  60.0f * normalized_time_3 +
                  30.0f * normalized_time_4) *
                 (1000.0f / (float)control->trajectory_duration_ms);
    control->target_position_rad =
        control->trajectory_start_position_rad * (1.0f - blend);
    control->target_velocity_rad_s =
        -control->trajectory_start_position_rad * blend_rate;
}

/* 功能：从当前位置启动 J4310 自动回零轨迹；用途：电机重连后平滑返回零位；无返回值表示控制权和轨迹状态已更新。 */
static void J4310AutoReturn_Start(j4310_auto_return_t *control,
                                  uint32_t tick_ms,
                                  float position_rad,
                                  float velocity_rad_s)
{
    control->reconnect_armed = false;
    control->owns_control = true;
    control->trajectory_start_ms = tick_ms;
    control->trajectory_start_position_rad = position_rad;
    control->trajectory_duration_ms =
        J4310AutoReturn_TrajectoryDurationMs(position_rad);
    control->target_position_rad = position_rad;
    control->target_velocity_rad_s = 0.0f;
    if ((fabsf(position_rad) <= J4310_AUTO_RETURN_SETTLE_POSITION_RAD) &&
        (fabsf(velocity_rad_s) <=
         J4310_AUTO_RETURN_SETTLE_VELOCITY_RAD_S))
    {
        control->stage = J4310_AUTO_RETURN_HOLDING;
        control->target_position_rad = 0.0f;
    }
    else
    {
        control->stage = J4310_AUTO_RETURN_RUNNING;
    }
}

/* 功能：初始化 J4310 自动回零控制器；用途：设置使能状态并清空历史状态；无返回值表示控制器已复位。 */
void J4310AutoReturn_Init(j4310_auto_return_t *control,
                          bool enabled)
{
    if (control == NULL)
    {
        return;
    }
    (void)memset(control, 0, sizeof(*control));
    control->enabled = enabled;
    control->stage = enabled ? J4310_AUTO_RETURN_ARMED :
                               J4310_AUTO_RETURN_DISABLED;
}

/* 功能：重新配置自动回零使能和反馈在线状态；用途：在系统启动或配置切换时重建状态机；无返回值表示配置已生效。 */
void J4310AutoReturn_Configure(j4310_auto_return_t *control,
                               bool enabled,
                               bool feedback_fresh)
{
    if (control == NULL)
    {
        return;
    }
    control->enabled = enabled;
    control->online = feedback_fresh;
    control->reconnect_armed = enabled && control->seen_online &&
                               !feedback_fresh;
    control->owns_control = false;
    control->target_position_rad = 0.0f;
    control->target_velocity_rad_s = 0.0f;
    control->stage = enabled ? J4310_AUTO_RETURN_ARMED :
                               J4310_AUTO_RETURN_DISABLED;
}

/* 功能：取消正在进行或等待中的自动回零；用途：在人工控制或停机时释放控制权；无返回值表示回零目标已清除。 */
void J4310AutoReturn_Cancel(j4310_auto_return_t *control)
{
    if (control == NULL)
    {
        return;
    }
    control->reconnect_armed = false;
    control->owns_control = false;
    control->target_position_rad = 0.0f;
    control->target_velocity_rad_s = 0.0f;
    control->stage = control->enabled ? J4310_AUTO_RETURN_ARMED :
                                        J4310_AUTO_RETURN_DISABLED;
}

/* 功能：按反馈在线状态推进自动回零状态机；用途：检测掉线重连并生成回零目标；无返回值表示本周期状态已更新。 */
void J4310AutoReturn_Update(j4310_auto_return_t *control,
                            uint32_t tick_ms,
                            bool feedback_fresh,
                            float position_rad,
                            float velocity_rad_s,
                            bool control_allowed)
{
    bool valid_feedback;
    bool was_online;

    if (control == NULL)
    {
        return;
    }
    valid_feedback = feedback_fresh && isfinite(position_rad) &&
                     isfinite(velocity_rad_s);
    if (!valid_feedback)
    {
        if (control->online || control->owns_control)
        {
            control->online = false;
            control->reconnect_armed = control->enabled &&
                                       control_allowed &&
                                       control->seen_online;
            control->owns_control = false;
        }
        control->stage = control->enabled ? J4310_AUTO_RETURN_ARMED :
                                            J4310_AUTO_RETURN_DISABLED;
        return;
    }

    was_online = control->online;
    control->online = true;
    if (!control->seen_online)
    {
        control->seen_online = true;
        control->reconnect_armed = false;
        control->owns_control = false;
        control->stage = control->enabled ? J4310_AUTO_RETURN_ARMED :
                                            J4310_AUTO_RETURN_DISABLED;
        return;
    }
    if (!control->enabled || !control_allowed)
    {
        control->reconnect_armed = false;
        control->owns_control = false;
        control->stage = control->enabled ? J4310_AUTO_RETURN_ARMED :
                                            J4310_AUTO_RETURN_DISABLED;
        return;
    }
    if (!was_online && control->reconnect_armed &&
        !control->owns_control)
    {
        J4310AutoReturn_Start(control,
                              tick_ms,
                              position_rad,
                              velocity_rad_s);
    }
    if (!control->owns_control)
    {
        return;
    }

    if ((fabsf(position_rad) <= J4310_AUTO_RETURN_SETTLE_POSITION_RAD) &&
        (fabsf(velocity_rad_s) <=
         J4310_AUTO_RETURN_SETTLE_VELOCITY_RAD_S))
    {
        control->stage = J4310_AUTO_RETURN_HOLDING;
        control->target_position_rad = 0.0f;
        control->target_velocity_rad_s = 0.0f;
        return;
    }
    if (control->stage == J4310_AUTO_RETURN_RUNNING)
    {
        J4310AutoReturn_Sample(control, tick_ms);
    }
}

/* 功能：判断自动回零控制器是否持有控制权；用途：决定上层是否采用回零目标；返回 true 表示自动回零正在生效。 */
bool J4310AutoReturn_IsActive(const j4310_auto_return_t *control)
{
    return (control != NULL) && control->owns_control;
}
