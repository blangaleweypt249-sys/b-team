#ifndef CONVEYOR_H
#define CONVEYOR_H

#include <stdbool.h>

#include "motor_manager.h"

typedef struct
{
    bool enabled;
    float m2006_vel_rad_s;
} conveyor_target_t;

typedef struct
{
    bool enabled;
    motor_cmd_t m2006;
} conveyor_output_t;

bool Conveyor_Calc(const conveyor_target_t *target,
                   conveyor_output_t *output);
bool Conveyor_Apply(motor_manager_t *manager,
                    const conveyor_output_t *output);

#endif
