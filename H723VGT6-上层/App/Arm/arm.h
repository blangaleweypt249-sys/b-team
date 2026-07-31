#ifndef ARM_H
#define ARM_H

#include <stdbool.h>

#include "motor_manager.h"

typedef struct
{
    bool enabled;
    float joint_pos_rad;
    float joint_vel_rad_s;
    float grip_pos_rad;
    float grip_vel_rad_s;
    float grip_kp;
    float grip_kd;
} arm_target_t;

void Arm_Update(motor_manager_t *manager, const arm_target_t *target);

#endif
