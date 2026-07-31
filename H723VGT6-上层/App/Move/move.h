#ifndef MOVE_H
#define MOVE_H

#include <stdbool.h>

#include "motor_manager.h"

typedef struct
{
    bool enabled;
    float m3508_vel_rad_s[2];
    float m2006_vel_rad_s[2];
} move_target_t;

void Move_Update(motor_manager_t *manager, const move_target_t *target);

#endif
