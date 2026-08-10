#ifndef CONVEYOR_H
#define CONVEYOR_H

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
} conveyor_target_t;

typedef struct
{
    bool enabled;
    motor_cmd_t m2006;
    bool pid_update;
    upper_pid_cfg_t m2006_speed_pid;
    upper_pid_cfg_t m2006_position_pid;
} conveyor_output_t;

bool Conveyor_Calc(const conveyor_target_t *target,
                   conveyor_output_t *output);
bool Conveyor_Apply(motor_manager_t *manager,
                    const conveyor_output_t *output);

#endif
