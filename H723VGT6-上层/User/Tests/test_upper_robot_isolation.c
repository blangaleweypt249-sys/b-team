/**
 * @file test_upper_robot_isolation.c
 * @brief 验证上层机器人各执行机构故障的隔离处理。
 */

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "upper_robot.h"

static bool arm_calc_result;
static bool arm_apply_result;
static bool gate_calc_result;
static bool gate_apply_result;
static bool gripper_calc_result;
static bool gripper_apply_result;
static uint32_t arm_apply_count;
static uint32_t gate_apply_count;
static uint32_t gripper_apply_count;
static uint32_t process_count;
static uint32_t stop_all_count;
static uint32_t disable_count[UPPER_MOTOR_COUNT];

static const motor_cfg_t test_motor_cfg[UPPER_MOTOR_COUNT] = {0};

/* 功能：提供测试用 CAN 发送桩并记录输出帧；用途：隔离真实硬件发送接口；返回 true 表示桩接受该帧。 */
static bool Test_Send(const motor_cfg_t *cfg /**< 当前电机的型号、总线及节点配置 */,
                      const motor_cmd_t *cmd /**< 测试发送桩接收的电机控制命令 */,
                      void *user_data /**< 调用回调函数时传递的用户上下文 */)
{
    (void)cfg;
    (void)cmd;
    (void)user_data;
    return true;
}

/* 功能：复位测试夹具、桩状态和调用计数；用途：保证各测试用例相互独立；无返回值表示测试环境已清空。 */
static void Test_Reset(void)
{
    arm_calc_result = true;
    arm_apply_result = true;
    gate_calc_result = true;
    gate_apply_result = true;
    gripper_calc_result = true;
    gripper_apply_result = true;
    arm_apply_count = 0U;
    gate_apply_count = 0U;
    gripper_apply_count = 0U;
    process_count = 0U;
    stop_all_count = 0U;
    (void)memset(disable_count, 0, sizeof(disable_count));
}

/* 功能：提供 MotorManager_Init 的测试桩实现；用途：隔离外部依赖并记录或模拟调用结果。 */
bool MotorManager_Init(motor_manager_t *manager /**< 需要操作的电机管理器 */,
                       const motor_cfg_t *cfg /**< 当前电机的型号、总线及节点配置 */,
                       size_t motor_count /**< 调用方提供的电机配置数量 */,
                       motor_send_t send /**< 电机管理器用于下发电机命令的回调函数 */,
                       void *user_data /**< 调用回调函数时传递的用户上下文 */)
{
    (void)cfg;
    (void)motor_count;
    (void)send;
    (void)user_data;
    (void)memset(manager, 0, sizeof(*manager));
    return true;
}

/* 功能：提供 MotorManager_SetEnabled 的测试桩实现；用途：隔离外部依赖并记录或模拟调用结果。 */
bool MotorManager_SetEnabled(motor_manager_t *manager /**< 需要操作的电机管理器 */,
                             size_t motor_index /**< 电机在管理器配置表中的下标 */,
                             bool enabled /**< 是否启用指定电机的管理器输出 */)
{
    assert(manager != NULL);
    assert(motor_index < UPPER_MOTOR_COUNT);
    if (!enabled)
    {
        disable_count[motor_index]++;
    }
    manager->enabled[motor_index] = enabled;
    return true;
}

/* 功能：提供 MotorManager_Process 的测试桩实现；用途：隔离外部依赖并记录或模拟调用结果。 */
void MotorManager_Process(motor_manager_t *manager /**< 需要操作的电机管理器 */, uint32_t tick_ms /**< 当前系统毫秒时刻 */)
{
    assert(manager != NULL);
    (void)tick_ms;
    process_count++;
}

/* 功能：提供 MotorManager_StopAll 的测试桩实现；用途：隔离外部依赖并记录或模拟调用结果。 */
void MotorManager_StopAll(motor_manager_t *manager /**< 需要操作的电机管理器 */)
{
    assert(manager != NULL);
    stop_all_count++;
}

/* 功能：提供 Arm_Calc 的测试桩实现；用途：隔离外部依赖并记录或模拟调用结果。 */
bool Arm_Calc(const arm_target_t *target /**< 本周期机械臂关节控制目标 */, arm_output_t *output /**< 用于写出机械臂电机命令的对象 */)
{
    assert(target != NULL);
    assert(output != NULL);
    return arm_calc_result;
}

/* 功能：提供 Arm_Apply 的测试桩实现；用途：隔离外部依赖并记录或模拟调用结果。 */
bool Arm_Apply(motor_manager_t *manager /**< 需要操作的电机管理器 */, const arm_output_t *output /**< 待下发的机械臂电机命令 */)
{
    assert(manager != NULL);
    assert(output != NULL);
    arm_apply_count++;
    return arm_apply_result;
}

/* 功能：提供 Gate_Calc 的测试桩实现；用途：隔离外部依赖并记录或模拟调用结果。 */
bool Gate_Calc(const gate_target_t *target /**< 本周期挡板机构控制目标 */, gate_output_t *output /**< 用于写出挡板 M2006 命令的对象 */)
{
    assert(target != NULL);
    assert(output != NULL);
    return gate_calc_result;
}

/* 功能：提供 Gate_Apply 的测试桩实现；用途：隔离外部依赖并记录或模拟调用结果。 */
bool Gate_Apply(motor_manager_t *manager /**< 需要操作的电机管理器 */, const gate_output_t *output /**< 待下发的挡板 M2006 电机命令 */)
{
    assert(manager != NULL);
    assert(output != NULL);
    gate_apply_count++;
    return gate_apply_result;
}

/* 功能：提供 Gripper_Calc 的测试桩实现；用途：隔离外部依赖并记录或模拟调用结果。 */
bool Gripper_Calc(const gripper_target_t *target /**< 本周期夹爪机构控制目标 */,
                  gripper_output_t *output /**< 用于写出夹爪 M2006 命令的对象 */)
{
    assert(target != NULL);
    assert(output != NULL);
    return gripper_calc_result;
}

/* 功能：提供 Gripper_Apply 的测试桩实现；用途：隔离外部依赖并记录或模拟调用结果。 */
bool Gripper_Apply(motor_manager_t *manager /**< 需要操作的电机管理器 */,
                   const gripper_output_t *output /**< 待下发的夹爪 M2006 电机命令 */)
{
    assert(manager != NULL);
    assert(output != NULL);
    gripper_apply_count++;
    return gripper_apply_result;
}

/* 功能：创建处于运行状态的机器人测试对象；用途：减少故障隔离用例的重复准备代码；返回值表示已初始化对象。 */
static upper_robot_t Test_CreateRunningRobot(void)
{
    upper_robot_t robot;

    assert(UpperRobot_Init(&robot,
                           test_motor_cfg,
                           UPPER_MOTOR_COUNT,
                           Test_Send,
                           NULL));
    UpperRobot_Start(&robot);
    assert(robot.state == ROBOT_RUN);
    return robot;
}

/* 功能：执行 StartupTargetsHoldDjiAtZero 场景测试；用途：验证对应输入下的行为和断言；无返回值表示由断言报告结果。 */
static void Test_StartupTargetsHoldDjiAtZero(void)
{
    upper_robot_t robot;

    Test_Reset();
    assert(UpperRobot_Init(&robot,
                           test_motor_cfg,
                           UPPER_MOTOR_COUNT,
                           Test_Send,
                           NULL));
    assert(robot.state == ROBOT_RUN);
    assert(robot.target.position_mode);
    assert(!robot.target.arm.enabled);
    assert(robot.target.arm.m3508_enabled);
    assert(robot.target.arm.position_mode);
    assert(robot.target.arm.m3508_pos_rad[0] == 0.0f);
    assert(robot.target.arm.m3508_pos_rad[1] == 0.0f);
    assert(robot.target.gate.enabled);
    assert(robot.target.gate.position_mode);
    assert(robot.target.gate.m2006_pos_rad == 0.0f);
    assert(robot.target.gripper.enabled);
    assert(robot.target.gripper.position_mode);
    assert(robot.target.gripper.m2006_pos_rad == 0.0f);
}

/* 功能：执行 ArmCalcFailureIsIsolated 场景测试；用途：验证对应输入下的行为和断言；无返回值表示由断言报告结果。 */
static void Test_ArmCalcFailureIsIsolated(void)
{
    upper_robot_t robot;

    Test_Reset();
    arm_calc_result = false;
    robot = Test_CreateRunningRobot();
    UpperRobot_Control1ms(&robot, 1U);

    assert(robot.state == ROBOT_RUN);
    assert(robot.control_count == 1U);
    assert(arm_apply_count == 0U);
    assert(gate_apply_count == 1U);
    assert(gripper_apply_count == 1U);
    assert(disable_count[UPPER_MOTOR_ARM_J4310] == 1U);
    assert(disable_count[UPPER_MOTOR_ARM_M3508_1] == 1U);
    assert(disable_count[UPPER_MOTOR_ARM_M3508_2] == 1U);
    assert(disable_count[UPPER_MOTOR_GATE_M2006] == 0U);
    assert(disable_count[UPPER_MOTOR_GRIPPER_M2006] == 0U);
    assert(process_count == 1U);
    assert(stop_all_count == 0U);
}

/* 功能：执行 GateApplyFailureIsIsolated 场景测试；用途：验证对应输入下的行为和断言；无返回值表示由断言报告结果。 */
static void Test_GateApplyFailureIsIsolated(void)
{
    upper_robot_t robot;

    Test_Reset();
    gate_apply_result = false;
    robot = Test_CreateRunningRobot();
    UpperRobot_Control1ms(&robot, 2U);

    assert(robot.state == ROBOT_RUN);
    assert(robot.control_count == 1U);
    assert(arm_apply_count == 1U);
    assert(gate_apply_count == 1U);
    assert(gripper_apply_count == 1U);
    assert(disable_count[UPPER_MOTOR_ARM_J4310] == 0U);
    assert(disable_count[UPPER_MOTOR_ARM_M3508_1] == 0U);
    assert(disable_count[UPPER_MOTOR_ARM_M3508_2] == 0U);
    assert(disable_count[UPPER_MOTOR_GATE_M2006] == 1U);
    assert(disable_count[UPPER_MOTOR_GRIPPER_M2006] == 0U);
    assert(process_count == 1U);
    assert(stop_all_count == 0U);
}

/* 功能：执行 GripperCalcFailureIsIsolated 场景测试；用途：验证对应输入下的行为和断言；无返回值表示由断言报告结果。 */
static void Test_GripperCalcFailureIsIsolated(void)
{
    upper_robot_t robot;

    Test_Reset();
    gripper_calc_result = false;
    robot = Test_CreateRunningRobot();
    UpperRobot_Control1ms(&robot, 3U);

    assert(robot.state == ROBOT_RUN);
    assert(robot.control_count == 1U);
    assert(arm_apply_count == 1U);
    assert(gate_apply_count == 1U);
    assert(gripper_apply_count == 0U);
    assert(disable_count[UPPER_MOTOR_ARM_J4310] == 0U);
    assert(disable_count[UPPER_MOTOR_ARM_M3508_1] == 0U);
    assert(disable_count[UPPER_MOTOR_ARM_M3508_2] == 0U);
    assert(disable_count[UPPER_MOTOR_GATE_M2006] == 0U);
    assert(disable_count[UPPER_MOTOR_GRIPPER_M2006] == 1U);
    assert(process_count == 1U);
    assert(stop_all_count == 0U);
}

/* 功能：执行 EStopStopsAndAcceptsFreshStart 场景测试；用途：验证对应输入下的行为和断言；无返回值表示由断言报告结果。 */
static void Test_EStopStopsAndAcceptsFreshStart(void)
{
    upper_robot_t robot;
    upper_target_t fresh_target;

    Test_Reset();
    robot = Test_CreateRunningRobot();
    robot.target.arm.enabled = true;
    UpperRobot_EStop(&robot);
    assert(robot.state == ROBOT_STOP);
    assert(stop_all_count == 1U);
    assert(!robot.target.arm.enabled);
    assert(!robot.target.arm.m3508_enabled);
    assert(!robot.target.gate.enabled);
    assert(!robot.target.gripper.enabled);

    UpperRobot_Control1ms(&robot, 4U);
    assert(process_count == 0U);

    fresh_target = robot.target;
    fresh_target.gate.enabled = true;
    UpperRobot_SetTarget(&robot, &fresh_target);
    UpperRobot_Start(&robot);
    assert(robot.state == ROBOT_RUN);
    UpperRobot_Control1ms(&robot, 5U);
    assert(process_count == 1U);
}

/* 功能：运行本文件的上层机器人各执行机构故障的隔离处理测试；用途：集中执行断言用例；返回 0 表示全部测试通过。 */
int main(void)
{
    Test_StartupTargetsHoldDjiAtZero();
    Test_ArmCalcFailureIsIsolated();
    Test_GateApplyFailureIsIsolated();
    Test_GripperCalcFailureIsIsolated();
    Test_EStopStopsAndAcceptsFreshStart();
    puts("upper robot isolation tests passed");
    return 0;
}
