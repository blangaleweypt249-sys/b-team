#ifndef CONVEYOR_H
#define CONVEYOR_H

#include <stdbool.h>

#include "motor_manager.h"

typedef struct
{
    bool enabled;
    float vel_rad_s[2];
} conveyor_target_t;

void Conveyor_Update(motor_manager_t *manager,
                     const conveyor_target_t *target);

#endif
