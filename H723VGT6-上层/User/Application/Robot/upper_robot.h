#ifndef UPPER_ROBOT_H
#define UPPER_ROBOT_H

#include <stdbool.h>
#include <stdint.h>

#include "arm.h"
#include "conveyor.h"
#include "gripper.h"
#include "robot_state.h"

typedef struct
{
    arm_target_t arm;
    conveyor_target_t conveyor;
    gripper_target_t gripper;
} upper_target_t;

typedef struct
{
    robot_state_t state;
    upper_target_t target;
    motor_manager_t motor_manager;
    uint32_t control_count;
} upper_robot_t;

bool UpperRobot_Init(upper_robot_t *robot,
                     motor_send_t send,
                     void *user_data);
void UpperRobot_SetTarget(upper_robot_t *robot,
                          const upper_target_t *target);
void UpperRobot_Start(upper_robot_t *robot);
void UpperRobot_Stop(upper_robot_t *robot);
void UpperRobot_EStop(upper_robot_t *robot);
void UpperRobot_ClearError(upper_robot_t *robot);
void UpperRobot_Control1ms(upper_robot_t *robot, uint32_t tick_ms);

#endif
