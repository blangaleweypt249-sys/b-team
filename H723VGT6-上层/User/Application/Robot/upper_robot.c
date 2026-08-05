#include "upper_robot.h"

#include <string.h>

#include "upper_config.h"

bool UpperRobot_Init(upper_robot_t *robot,
                     motor_send_t send,
                     void *user_data)
{
    if (robot == NULL)
    {
        return false;
    }

    (void)memset(robot, 0, sizeof(*robot));
    if (!MotorManager_Init(&robot->motor_manager,
                           upper_motor_cfg,
                           UPPER_MOTOR_COUNT,
                           send,
                           user_data))
    {
        robot->state = ROBOT_ERROR;
        return false;
    }

    robot->state = ROBOT_READY;
    return true;
}

void UpperRobot_SetTarget(upper_robot_t *robot,
                          const upper_target_t *target)
{
    if ((robot != NULL) && (target != NULL))
    {
        robot->target = *target;
    }
}

void UpperRobot_Start(upper_robot_t *robot)
{
    if ((robot != NULL) &&
        ((robot->state == ROBOT_READY) || (robot->state == ROBOT_STOP)))
    {
        robot->state = ROBOT_RUN;
    }
}

void UpperRobot_Stop(upper_robot_t *robot)
{
    if (robot == NULL)
    {
        return;
    }

    MotorManager_StopAll(&robot->motor_manager);
    robot->state = ROBOT_STOP;
}

void UpperRobot_EStop(upper_robot_t *robot)
{
    if (robot == NULL)
    {
        return;
    }

    MotorManager_StopAll(&robot->motor_manager);
    robot->state = ROBOT_ERROR;
}

void UpperRobot_ClearError(upper_robot_t *robot)
{
    if ((robot != NULL) && (robot->state == ROBOT_ERROR))
    {
        robot->state = ROBOT_READY;
    }
}

void UpperRobot_Control1ms(upper_robot_t *robot, uint32_t tick_ms)
{
    arm_output_t arm_output;
    conveyor_output_t conveyor_output;
    gripper_output_t gripper_output;

    if ((robot == NULL) || (robot->state != ROBOT_RUN))
    {
        return;
    }

    /* Fixed order: state/targets -> module control -> CAN scheduling. */
    if (!Arm_Calc(&robot->target.arm, &arm_output) ||
        !Conveyor_Calc(&robot->target.conveyor, &conveyor_output) ||
        !Gripper_Calc(&robot->target.gripper, &gripper_output) ||
        !Arm_Apply(&robot->motor_manager, &arm_output) ||
        !Conveyor_Apply(&robot->motor_manager, &conveyor_output) ||
        !Gripper_Apply(&robot->motor_manager, &gripper_output))
    {
        UpperRobot_EStop(robot);
        return;
    }

    MotorManager_Process(&robot->motor_manager, tick_ms);
    robot->control_count++;
}
