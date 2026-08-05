#ifndef ARM_H
#define ARM_H

#include <stdbool.h>

#include "motor_manager.h"

typedef struct
{
    bool enabled;
    float grip_pos_rad;
    float grip_vel_rad_s;
    float grip_kp;
    float grip_kd;
    float m3508_vel_rad_s[4];
} arm_target_t;

typedef struct
{
    bool enabled;
    motor_cmd_t j4310;
    motor_cmd_t m3508[4];
} arm_output_t;

bool Arm_Calc(const arm_target_t *target, arm_output_t *output);
bool Arm_Apply(motor_manager_t *manager, const arm_output_t *output);

#endif
