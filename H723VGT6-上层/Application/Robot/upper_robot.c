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
    if ((robot == NULL) || (robot->state != ROBOT_RUN))
    {
        return;
    }

    /* Fixed order: state/targets -> module control -> CAN scheduling. */
    Arm_Update(&robot->motor_manager, &robot->target.arm);
    Move_Update(&robot->motor_manager, &robot->target.move);
    Ore_Update(&robot->motor_manager, &robot->target.ore);
    Gate_Update(&robot->motor_manager, &robot->target.gate);
    Conveyor_Update(&robot->motor_manager, &robot->target.conveyor);
    MotorManager_Process(&robot->motor_manager, tick_ms);
    robot->control_count++;
}
