#ifndef UPPER_CONFIG_H
#define UPPER_CONFIG_H

#include <stddef.h>

#include "motor_manager.h"

#define UPPER_CONTROL_PERIOD_MS 1U
#define UPPER_PC_TIMEOUT_MS     200U
#define UPPER_MOTOR_FEEDBACK_TIMEOUT_MS       50U

#define UPPER_MG5010_CONTROL_PERIOD_MS         5U
#define UPPER_MG5010_CURRENT_LIMIT_A           3.0f
/* Outer-loop gains use output-shaft rad/s and produce current in A. */
#define UPPER_MG5010_SPEED_KP                  2.291831181f
#define UPPER_MG5010_SPEED_KI                  5.729577951f
#define UPPER_MG5010_SPEED_KD                  0.028647890f
#define UPPER_MG5010_SPEED_I_LIMIT             0.139626340f

/* These MIT mapping limits must match the values stored in the J4310. */
#define UPPER_J4310_POSITION_MAX_RAD           12.5f
#define UPPER_J4310_VELOCITY_MAX_RAD_S         30.0f
#define UPPER_J4310_TORQUE_MAP_MAX_NM          10.0f

/* Conservative initial M3508 limits. Tune the PID values on the mechanism. */
#define UPPER_M3508_CURRENT_LIMIT_A             3.0f
#define UPPER_M3508_POSITION_VEL_LIMIT_RAD_S    15.708f
#define UPPER_M3508_SPEED_KP                    0.9325f
#define UPPER_M3508_SPEED_KI                    0.4663f
#define UPPER_M3508_SPEED_KD                    0.0f
#define UPPER_M3508_SPEED_I_LIMIT               6.0f
#define UPPER_M3508_POSITION_KP                 6.2832f
#define UPPER_M3508_POSITION_KI                 0.0f
#define UPPER_M3508_POSITION_KD                 0.0f
#define UPPER_M3508_POSITION_I_LIMIT            0.0f

/* Conservative initial M2006/C610 limits. Tune on the real mechanism. */
#define UPPER_M2006_CURRENT_LIMIT_A             2.0f
#define UPPER_M2006_POSITION_VEL_LIMIT_RAD_S    10.472f
#define UPPER_M2006_SPEED_KP                    0.2865f
#define UPPER_M2006_SPEED_KI                    0.0955f
#define UPPER_M2006_SPEED_KD                    0.0f
#define UPPER_M2006_SPEED_I_LIMIT               10.0f
#define UPPER_M2006_POSITION_KP                 4.1888f
#define UPPER_M2006_POSITION_KI                 0.0f
#define UPPER_M2006_POSITION_KD                 0.0f
#define UPPER_M2006_POSITION_I_LIMIT            0.0f

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
