#include "upper_robot.h"

#include <string.h>

#include "upper_config.h"

/* 功能：初始化上层机器人对象和电机管理器；用途：建立控制状态与发送接口；返回 true 表示机器人进入就绪态。 */
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

/* 功能：保存一份新的整机控制目标；用途：供后续 1 ms 控制周期统一读取；无返回值表示覆盖目标快照。 */
void UpperRobot_SetTarget(upper_robot_t *robot,
                          const upper_target_t *target)
{
    if ((robot != NULL) && (target != NULL))
    {
        robot->target = *target;
    }
}

/* 功能：将就绪或停止状态切换为运行；用途：允许周期控制开始下发电机命令；无返回值表示仅执行合法状态转换。 */
void UpperRobot_Start(upper_robot_t *robot)
{
    if ((robot != NULL) &&
        ((robot->state == ROBOT_READY) || (robot->state == ROBOT_STOP)))
    {
        robot->state = ROBOT_RUN;
    }
}

/* 功能：停止全部电机并进入停止态；用途：执行正常停机；无返回值表示状态和电机均被更新。 */
void UpperRobot_Stop(upper_robot_t *robot)
{
    if (robot == NULL)
    {
        return;
    }

    MotorManager_StopAll(&robot->motor_manager);
    robot->state = ROBOT_STOP;
}

/* 功能：紧急停止全部电机并进入错误态；用途：处理控制或通信异常；无返回值表示系统被锁定等待清错。 */
void UpperRobot_EStop(upper_robot_t *robot)
{
    if (robot == NULL)
    {
        return;
    }

    MotorManager_StopAll(&robot->motor_manager);
    robot->state = ROBOT_ERROR;
}

/* 功能：把错误态恢复为就绪态；用途：故障排除后重新允许启动；无返回值表示仅在当前为错误态时生效。 */
void UpperRobot_ClearError(upper_robot_t *robot)
{
    if ((robot != NULL) && (robot->state == ROBOT_ERROR))
    {
        robot->state = ROBOT_READY;
    }
}

/* 功能：执行整机 1 ms 控制周期；用途：计算各机构命令、提交电机并统一发送；异常时会触发急停。 */
void UpperRobot_Control1ms(upper_robot_t *robot, uint32_t tick_ms)
{
    arm_output_t arm_output;
    conveyor_output_t conveyor_output;
    gripper_output_t gripper_output;

    if ((robot == NULL) || (robot->state != ROBOT_RUN))
    {
        return;
    }

    /* Build one target snapshot, stage every module, then dispatch CAN frames
     * only after J4310/M3508/M2006 targets are all ready for this 1 ms cycle. */
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
