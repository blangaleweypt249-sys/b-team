#include "upper_robot.h"

#include <string.h>

#include "upper_config.h"

/* 功能：初始化上层机器人对象和电机管理器；用途：让 DJI 电机以上电位置零点进入位控；返回 true 表示机器人进入运行态。 */
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

    robot->target.position_mode = true;
    robot->target.arm.position_mode = true;
    robot->target.arm.m3508_enabled = true;
    robot->target.conveyor.position_mode = true;
    robot->target.conveyor.enabled = true;
    robot->target.gripper.position_mode = true;
    robot->target.gripper.enabled = true;
    robot->state = ROBOT_RUN;
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

/* 功能：紧急停止全部电机并进入停止态；用途：立即撤销输出并等待新的显式控制目标。 */
void UpperRobot_EStop(upper_robot_t *robot)
{
    if (robot == NULL)
    {
        return;
    }

    MotorManager_StopAll(&robot->motor_manager);
    robot->target.arm.enabled = false;
    robot->target.arm.m3508_enabled = false;
    robot->target.arm.j4310_commanded = false;
    robot->target.conveyor.enabled = false;
    robot->target.gripper.enabled = false;
    robot->state = ROBOT_STOP;
}

/* 功能：把错误态恢复为就绪态；用途：故障排除后重新允许启动；无返回值表示仅在当前为错误态时生效。 */
void UpperRobot_ClearError(upper_robot_t *robot)
{
    if ((robot != NULL) && (robot->state == ROBOT_ERROR))
    {
        robot->state = ROBOT_READY;
    }
}

/* 功能：关闭机械臂全部电机；用途：把机械臂局部计算或应用失败限制在本机构内。 */
static void UpperRobot_DisableArm(motor_manager_t *manager)
{
    uint32_t index;

    (void)MotorManager_SetEnabled(manager,
                                  UPPER_MOTOR_ARM_J4310,
                                  false);
    for (index = 0U; index < UPPER_ARM_M3508_COUNT; index++)
    {
        (void)MotorManager_SetEnabled(
            manager,
            (size_t)UPPER_MOTOR_ARM_M3508_1 + index,
            false);
    }
}

/* 功能：关闭传送机构电机；用途：阻止无效传送目标继续使用旧命令。 */
static void UpperRobot_DisableConveyor(motor_manager_t *manager)
{
    (void)MotorManager_SetEnabled(manager,
                                  UPPER_MOTOR_CONVEYOR_M2006,
                                  false);
}

/* 功能：关闭夹爪电机；用途：阻止无效夹爪目标继续使用旧命令。 */
static void UpperRobot_DisableGripper(motor_manager_t *manager)
{
    (void)MotorManager_SetEnabled(manager,
                                  UPPER_MOTOR_GRIPPER_M2006,
                                  false);
}

/* 功能：执行整机 1 ms 控制周期；用途：独立计算并提交各机构命令；单个机构异常时仅关闭对应电机。 */
void UpperRobot_Control1ms(upper_robot_t *robot, uint32_t tick_ms)
{
    arm_output_t arm_output;
    conveyor_output_t conveyor_output;
    gripper_output_t gripper_output;

    if ((robot == NULL) || (robot->state != ROBOT_RUN))
    {
        return;
    }

    if (!Arm_Calc(&robot->target.arm, &arm_output) ||
        !Arm_Apply(&robot->motor_manager, &arm_output))
    {
        UpperRobot_DisableArm(&robot->motor_manager);
    }
    if (!Conveyor_Calc(&robot->target.conveyor, &conveyor_output) ||
        !Conveyor_Apply(&robot->motor_manager, &conveyor_output))
    {
        UpperRobot_DisableConveyor(&robot->motor_manager);
    }
    if (!Gripper_Calc(&robot->target.gripper, &gripper_output) ||
        !Gripper_Apply(&robot->motor_manager, &gripper_output))
    {
        UpperRobot_DisableGripper(&robot->motor_manager);
    }

    MotorManager_Process(&robot->motor_manager, tick_ms);
    robot->control_count++;
}
