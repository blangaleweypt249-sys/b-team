#ifndef UPPER_CONFIG_H
#define UPPER_CONFIG_H

#include <stddef.h>

#include "motor_manager.h"

#define UPPER_CONTROL_FREQUENCY_HZ             1000U
#define UPPER_CONTROL_PERIOD_MS                1U
#define UPPER_ARM_M3508_COUNT                  2U
#define UPPER_CONTROL_WATCHDOGS_ENABLED        0U
#define UPPER_PC_TIMEOUT_MS                    200U
#define UPPER_MOTOR_FEEDBACK_TIMEOUT_MS        50U

/* These MIT mapping limits must match the values stored in the J4310. */
#define UPPER_J4310_POSITION_MAX_RAD           12.5f
#define UPPER_J4310_VELOCITY_MAX_RAD_S         30.0f
#define UPPER_J4310_TORQUE_MAP_MAX_NM          10.0f
/* Positive mechanism motion is opposite to the J4310 protocol direction. */
#define UPPER_J4310_DIRECTION_SIGN              (-1.0f)
#define UPPER_J4310_TRAJECTORY_MAX_VEL_RAD_S     5.0f
#define UPPER_J4310_TRAJECTORY_MAX_ACCEL_RAD_S2 30.0f
#define UPPER_J4310_GRAVITY_MODEL_LIMIT_NM       8.0f
#define UPPER_J4310_GRAVITY_LEARNING_RATE        0.01f

/* Fixed production limits. The PC angle command cannot replace these PIDs. */
/* Equal mechanism targets require opposite M3508 protocol directions. */
#define UPPER_M3508_1_DIRECTION_SIGN            (1.0f)
#define UPPER_M3508_2_DIRECTION_SIGN            (-1.0f)
#define UPPER_M3508_CURRENT_LIMIT_A             3.0f
#define UPPER_M3508_POSITION_VEL_LIMIT_RAD_S    15.708f
#define UPPER_M3508_POSITION_PID_OUTPUT_LIMIT_RAD_S \
    UPPER_M3508_POSITION_VEL_LIMIT_RAD_S
#define UPPER_M3508_ACCEL_LIMIT_RAD_S2          62.832f
#define UPPER_M3508_SPEED_KP                    0.9325f
#define UPPER_M3508_SPEED_KI                    0.4663f
#define UPPER_M3508_SPEED_KD                    0.0f
#define UPPER_M3508_SPEED_I_LIMIT               6.0f
#define UPPER_M3508_POSITION_KP                 6.2832f
#define UPPER_M3508_POSITION_KI                 0.0f
#define UPPER_M3508_POSITION_KD                 0.0f
#define UPPER_M3508_POSITION_I_LIMIT            0.0f

#define UPPER_M2006_CURRENT_LIMIT_A             2.0f
#define UPPER_M2006_POSITION_LIMIT_RAD          6.28318530718f
#define UPPER_M2006_POSITION_EPSILON_RAD        0.000001f
#define UPPER_M2006_POSITION_CUTOFF_RAD         6.45771823238f
/* The gripper has an additional 2:1 reduction after the C610 output. */
#define UPPER_GRIPPER_M2006_POSITION_LIMIT_RAD  12.56637061436f
#define UPPER_GRIPPER_M2006_POSITION_CUTOFF_RAD 12.74090353956f
#define UPPER_M2006_POSITION_VEL_LIMIT_RAD_S    10.472f
#define UPPER_M2006_POSITION_PID_OUTPUT_LIMIT_RAD_S \
    UPPER_M2006_POSITION_VEL_LIMIT_RAD_S
#define UPPER_M2006_ACCEL_LIMIT_RAD_S2          62.832f
#define UPPER_M2006_SPEED_KP                    0.2865f
#define UPPER_M2006_SPEED_KI                    0.0955f
#define UPPER_M2006_SPEED_KD                    0.0f
#define UPPER_M2006_SPEED_I_LIMIT               10.0f
#define UPPER_M2006_POSITION_KP                 4.1888f
#define UPPER_M2006_POSITION_KI                 0.0f
#define UPPER_M2006_POSITION_KD                 0.0f
#define UPPER_M2006_POSITION_I_LIMIT            0.0f
/* Positive gate motion is opposite to the C610 protocol direction. */
#define UPPER_GATE_M2006_DIRECTION_SIGN         (-1.0f)

/* Physical remote action targets, in mechanism output degrees. */
#define UPPER_REMOTE_PD13_FIRST_M3508_DEG       500.0f
#define UPPER_REMOTE_PD13_FIRST_J4310_DEG        90.0f
#define UPPER_REMOTE_PD13_SECOND_M3508_DEG        0.0f
#define UPPER_REMOTE_PD13_SECOND_J4310_DEG      (-20.0f)
#define UPPER_REMOTE_PD13_RESET_DELAY_MS       1000U
#define UPPER_REMOTE_PD12_FIRST_M3508_DEG      1000.0f
#define UPPER_REMOTE_PD12_FIRST_J4310_DEG        90.0f
#define UPPER_REMOTE_PD12_SECOND_M3508_DEG        0.0f
#define UPPER_REMOTE_PD12_SECOND_J4310_DEG      180.0f
#define UPPER_REMOTE_PD11_FIRST_M3508_DEG         0.0f
#define UPPER_REMOTE_PD11_FIRST_J4310_DEG        90.0f
#define UPPER_REMOTE_PD11_SECOND_M3508_DEG        0.0f
#define UPPER_REMOTE_PD11_SECOND_J4310_DEG      180.0f
#define UPPER_REMOTE_PD8_FIRST_M3508_DEG          0.0f
#define UPPER_REMOTE_PD8_FIRST_J4310_DEG         30.0f
#define UPPER_REMOTE_PD8_FIRST_DELAY_MS          1000U
#define UPPER_REMOTE_PD8_SECOND_M3508_DEG       700.0f
#define UPPER_REMOTE_PD8_SECOND_J4310_DEG        90.0f
#define UPPER_REMOTE_PD8_THIRD_M3508_DEG          0.0f
#define UPPER_REMOTE_PD8_THIRD_J4310_DEG        180.0f
#define UPPER_REMOTE_PD9_FIRST_GATE_DEG          180.0f
#define UPPER_REMOTE_PD9_SECOND_GATE_DEG          62.0f
#define UPPER_REMOTE_PD10_FIRST_GRIPPER_DEG      130.0f
#define UPPER_REMOTE_PD10_SECOND_GRIPPER_DEG      55.0f

#define UPPER_J4310_KP_MAX                      500.0f
#define UPPER_J4310_KD_MAX                      5.0f

typedef enum
{
    UPPER_MOTOR_ARM_M3508_1,
    UPPER_MOTOR_ARM_M3508_2,
    UPPER_MOTOR_ARM_J4310,
    UPPER_MOTOR_CONVEYOR_M2006,
    UPPER_MOTOR_GRIPPER_M2006,
    UPPER_MOTOR_COUNT
} upper_motor_id_t;

#define UPPER_MOTOR_GATE_M2006 UPPER_MOTOR_CONVEYOR_M2006

extern const motor_cfg_t upper_motor_cfg[UPPER_MOTOR_COUNT];

#endif
