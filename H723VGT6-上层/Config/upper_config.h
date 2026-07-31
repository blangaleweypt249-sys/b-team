#ifndef UPPER_CONFIG_H
#define UPPER_CONFIG_H

#include <stddef.h>

#include "motor_manager.h"

#define UPPER_CONTROL_PERIOD_MS 1U
#define UPPER_PC_TIMEOUT_MS     200U

typedef enum
{
    UPPER_MOTOR_ARM_MG5010,
    UPPER_MOTOR_ARM_J4310,
    UPPER_MOTOR_MOVE_M3508_L,
    UPPER_MOTOR_MOVE_M3508_R,
    UPPER_MOTOR_MOVE_M2006_L,
    UPPER_MOTOR_MOVE_M2006_R,
    UPPER_MOTOR_ORE_M2006,
    UPPER_MOTOR_GATE_M2006,
    UPPER_MOTOR_CONVEYOR_L,
    UPPER_MOTOR_CONVEYOR_R,
    UPPER_MOTOR_COUNT
} upper_motor_id_t;

extern const motor_cfg_t upper_motor_cfg[UPPER_MOTOR_COUNT];

#endif
