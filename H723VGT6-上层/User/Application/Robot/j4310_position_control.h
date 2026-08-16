#ifndef J4310_POSITION_CONTROL_H
#define J4310_POSITION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool initialized;
    bool trajectory_active;
    bool feedback_seen;
    bool gravity_position_valid;
    uint32_t trajectory_start_ms;
    uint32_t trajectory_duration_ms;
    uint32_t last_feedback_ms;
    float trajectory_start_position_rad;
    float trajectory_target_position_rad;
    float target_position_rad;
    float target_velocity_rad_s;
    float max_velocity_rad_s;
    float max_acceleration_rad_s2;
    float gravity_model_limit_nm;
    float gravity_learning_rate;
    float gravity_reference_position_rad;
    float gravity_cos_nm;
    float gravity_sin_nm;
    float gravity_torque_nm;
} j4310_position_control_t;

bool J4310PositionControl_Init(j4310_position_control_t *control,
                               float max_velocity_rad_s,
                               float max_acceleration_rad_s2,
                               float gravity_model_limit_nm,
                               float gravity_learning_rate);
bool J4310PositionControl_Start(j4310_position_control_t *control,
                                uint32_t tick_ms,
                                float start_position_rad,
                                float target_position_rad);
void J4310PositionControl_Hold(j4310_position_control_t *control,
                               float position_rad);
void J4310PositionControl_CancelTrajectory(
    j4310_position_control_t *control);
void J4310PositionControl_Sample(j4310_position_control_t *control,
                                 uint32_t tick_ms,
                                 float *position_rad,
                                 float *velocity_rad_s);
float J4310PositionControl_ComposeTorque(
    j4310_position_control_t *control,
    bool feedback_fresh,
    uint32_t feedback_ms,
    float actual_position_rad,
    float actual_velocity_rad_s,
    float feedback_torque_nm,
    float desired_position_rad,
    float desired_velocity_rad_s,
    float requested_torque_nm,
    float torque_limit_nm);

#endif
