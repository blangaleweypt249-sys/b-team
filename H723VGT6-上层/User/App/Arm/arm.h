#ifndef ARM_H
#define ARM_H

#include <stdbool.h>

#include "upper_config.h"
#include "upper_pid.h"

typedef struct
{
    bool enabled;
    bool position_mode;
    float grip_pos_rad;
    float grip_vel_rad_s;
    float grip_kp;
    float grip_kd;
    float grip_torque_nm;
    float grip_torque_limit_nm;
    float m3508_vel_rad_s[UPPER_ARM_M3508_COUNT];
    float m3508_pos_rad[UPPER_ARM_M3508_COUNT];
    bool pid_update;
    upper_pid_cfg_t m3508_speed_pid;
    upper_pid_cfg_t m3508_position_pid;
} arm_target_t;

typedef struct
{
    bool enabled;
    motor_cmd_t j4310;
    motor_cmd_t m3508[UPPER_ARM_M3508_COUNT];
    bool pid_update;
    float j4310_torque_limit_nm;
    upper_pid_cfg_t m3508_speed_pid;
    upper_pid_cfg_t m3508_position_pid;
} arm_output_t;

bool Arm_Calc(const arm_target_t *target, arm_output_t *output);
bool Arm_Apply(motor_manager_t *manager, const arm_output_t *output);

#endif
