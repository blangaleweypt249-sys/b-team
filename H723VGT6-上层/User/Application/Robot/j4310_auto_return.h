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

void J4310AutoReturn_Init(j4310_auto_return_t *control,
                          bool enabled);
void J4310AutoReturn_Configure(j4310_auto_return_t *control,
                               bool enabled,
                               bool feedback_fresh);
void J4310AutoReturn_Cancel(j4310_auto_return_t *control);
void J4310AutoReturn_Update(j4310_auto_return_t *control,
                            uint32_t tick_ms,
                            bool feedback_fresh,
                            float position_rad,
                            float velocity_rad_s,
                            bool control_allowed);

#endif
