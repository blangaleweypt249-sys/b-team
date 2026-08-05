#ifndef GRIPPER_H
#define GRIPPER_H

#include <stdbool.h>

#include "motor_manager.h"

typedef struct
{
    bool enabled;
    float m2006_vel_rad_s;
} gripper_target_t;

typedef struct
{
    bool enabled;
    motor_cmd_t m2006;
} gripper_output_t;

bool Gripper_Calc(const gripper_target_t *target,
                  gripper_output_t *output);
bool Gripper_Apply(motor_manager_t *manager,
                   const gripper_output_t *output);

#endif
