#include "j4310_auto_return.h"

#include <math.h>
#include <string.h>

#define J4310_AUTO_RETURN_CONTROL_PERIOD_MS       10U
#define J4310_AUTO_RETURN_MAX_VELOCITY_RAD_S       2.0f
#define J4310_AUTO_RETURN_MAX_ACCEL_RAD_S2         5.0f
#define J4310_AUTO_RETURN_SETTLE_POSITION_RAD      0.03f
#define J4310_AUTO_RETURN_SETTLE_VELOCITY_RAD_S    0.20f
#define J4310_AUTO_RETURN_QUINTIC_VELOCITY_BOUND   1.875f
#define J4310_AUTO_RETURN_QUINTIC_ACCEL_BOUND      5.7736f

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

void J4310AutoReturn_Init(j4310_auto_return_t *control,
                          bool enabled)
{
    if (control == NULL)
    {
        return;
    }
    (void)memset(control, 0, sizeof(*control));
    control->enabled = enabled;
    control->reconnect_armed = enabled;
    control->stage = enabled ? J4310_AUTO_RETURN_ARMED :
                               J4310_AUTO_RETURN_DISABLED;
}

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
    control->reconnect_armed = enabled && !feedback_fresh;
    control->owns_control = false;
    control->target_position_rad = 0.0f;
    control->target_velocity_rad_s = 0.0f;
    control->stage = enabled ? J4310_AUTO_RETURN_ARMED :
                               J4310_AUTO_RETURN_DISABLED;
}

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

void J4310AutoReturn_Update(j4310_auto_return_t *control,
                            uint32_t tick_ms,
                            bool feedback_fresh,
                            float position_rad,
                            float velocity_rad_s,
                            bool control_allowed)
{
    if (control == NULL)
    {
        return;
    }
    if (!control->enabled)
    {
        control->online = feedback_fresh;
        control->reconnect_armed = false;
        control->owns_control = false;
        control->stage = J4310_AUTO_RETURN_DISABLED;
        return;
    }
    if (!control_allowed)
    {
        control->owns_control = false;
        control->stage = J4310_AUTO_RETURN_ARMED;
        return;
    }
    if (!feedback_fresh || !isfinite(position_rad) ||
        !isfinite(velocity_rad_s))
    {
        if (control->online || control->owns_control)
        {
            control->online = false;
            control->reconnect_armed = true;
            control->owns_control = false;
            control->stage = J4310_AUTO_RETURN_ARMED;
        }
        return;
    }

    if (!control->online)
    {
        control->online = true;
    }
    if (control->reconnect_armed && !control->owns_control)
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
