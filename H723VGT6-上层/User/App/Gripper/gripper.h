#ifndef GRIPPER_H
#define GRIPPER_H

#include <stdbool.h>

#include "motor_manager.h"
#include "upper_pid.h"

typedef struct
{
    bool enabled;
    bool position_mode;
    float m2006_vel_rad_s;
    float m2006_pos_rad;
    bool pid_update;
    upper_pid_cfg_t m2006_speed_pid;
    upper_pid_cfg_t m2006_position_pid;
} gripper_target_t;

typedef struct
{
    bool enabled;
    motor_cmd_t m2006;
    bool pid_update;
    upper_pid_cfg_t m2006_speed_pid;
    upper_pid_cfg_t m2006_position_pid;
} gripper_output_t;

bool Gripper_Calc(const gripper_target_t *target,
                  gripper_output_t *output);
bool Gripper_Apply(motor_manager_t *manager,
                   const gripper_output_t *output);

#endif
