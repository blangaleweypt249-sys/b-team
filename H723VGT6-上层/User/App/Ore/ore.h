#ifndef ORE_H
#define ORE_H

#include <stdbool.h>

#include "motor_manager.h"

typedef struct
{
    bool enabled;
    float vel_rad_s;
} ore_target_t;

void Ore_Update(motor_manager_t *manager, const ore_target_t *target);

#endif
