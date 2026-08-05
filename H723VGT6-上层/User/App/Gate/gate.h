#ifndef GATE_H
#define GATE_H

#include <stdbool.h>

#include "motor_manager.h"

typedef struct
{
    bool enabled;
    float vel_rad_s;
} gate_target_t;

void Gate_Update(motor_manager_t *manager, const gate_target_t *target);

#endif
