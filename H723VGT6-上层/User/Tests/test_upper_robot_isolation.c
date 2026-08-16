#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "upper_config.h"
#include "upper_robot.h"

static bool arm_calc_result;
static bool arm_apply_result;
static bool conveyor_calc_result;
static bool conveyor_apply_result;
static bool gripper_calc_result;
static bool gripper_apply_result;
static uint32_t arm_apply_count;
static uint32_t conveyor_apply_count;
static uint32_t gripper_apply_count;
static uint32_t process_count;
static uint32_t stop_all_count;
static uint32_t disable_count[UPPER_MOTOR_COUNT];

const motor_cfg_t upper_motor_cfg[UPPER_MOTOR_COUNT] = {0};

static bool Test_Send(const motor_cfg_t *cfg,
                      const motor_cmd_t *cmd,
                      void *user_data)
{
    (void)cfg;
    (void)cmd;
    (void)user_data;
    return true;
}

static void Test_Reset(void)
{
    arm_calc_result = true;
    arm_apply_result = true;
    conveyor_calc_result = true;
    conveyor_apply_result = true;
    gripper_calc_result = true;
    gripper_apply_result = true;
    arm_apply_count = 0U;
    conveyor_apply_count = 0U;
    gripper_apply_count = 0U;
    process_count = 0U;
    stop_all_count = 0U;
    (void)memset(disable_count, 0, sizeof(disable_count));
}

bool MotorManager_Init(motor_manager_t *manager,
                       const motor_cfg_t *cfg,
                       size_t motor_count,
                       motor_send_t send,
                       void *user_data)
{
    (void)cfg;
    (void)motor_count;
    (void)send;
    (void)user_data;
    (void)memset(manager, 0, sizeof(*manager));
    return true;
}

bool MotorManager_SetEnabled(motor_manager_t *manager,
                             size_t motor_index,
                             bool enabled)
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

void MotorManager_Process(motor_manager_t *manager, uint32_t tick_ms)
{
    assert(manager != NULL);
    (void)tick_ms;
    process_count++;
}

void MotorManager_StopAll(motor_manager_t *manager)
{
    assert(manager != NULL);
    stop_all_count++;
}

bool Arm_Calc(const arm_target_t *target, arm_output_t *output)
{
    assert(target != NULL);
    assert(output != NULL);
    return arm_calc_result;
}

bool Arm_Apply(motor_manager_t *manager, const arm_output_t *output)
{
    assert(manager != NULL);
    assert(output != NULL);
    arm_apply_count++;
    return arm_apply_result;
}

bool Conveyor_Calc(const conveyor_target_t *target,
                   conveyor_output_t *output)
{
    assert(target != NULL);
    assert(output != NULL);
    return conveyor_calc_result;
}

bool Conveyor_Apply(motor_manager_t *manager,
                    const conveyor_output_t *output)
{
    assert(manager != NULL);
    assert(output != NULL);
    conveyor_apply_count++;
    return conveyor_apply_result;
}

bool Gripper_Calc(const gripper_target_t *target,
                  gripper_output_t *output)
{
    assert(target != NULL);
    assert(output != NULL);
    return gripper_calc_result;
}

bool Gripper_Apply(motor_manager_t *manager,
                   const gripper_output_t *output)
{
    assert(manager != NULL);
    assert(output != NULL);
    gripper_apply_count++;
    return gripper_apply_result;
}

static upper_robot_t Test_CreateRunningRobot(void)
{
    upper_robot_t robot;

    assert(UpperRobot_Init(&robot, Test_Send, NULL));
    UpperRobot_Start(&robot);
    assert(robot.state == ROBOT_RUN);
    return robot;
}

static void Test_StartupTargetsHoldDjiAtZero(void)
{
    upper_robot_t robot;

    Test_Reset();
    assert(UpperRobot_Init(&robot, Test_Send, NULL));
    assert(robot.state == ROBOT_RUN);
    assert(robot.target.position_mode);
    assert(!robot.target.arm.enabled);
    assert(robot.target.arm.m3508_enabled);
    assert(robot.target.arm.position_mode);
    assert(robot.target.arm.m3508_pos_rad[0] == 0.0f);
    assert(robot.target.arm.m3508_pos_rad[1] == 0.0f);
    assert(robot.target.conveyor.enabled);
    assert(robot.target.conveyor.position_mode);
    assert(robot.target.conveyor.m2006_pos_rad == 0.0f);
    assert(robot.target.gripper.enabled);
    assert(robot.target.gripper.position_mode);
    assert(robot.target.gripper.m2006_pos_rad == 0.0f);
}

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
    assert(conveyor_apply_count == 1U);
    assert(gripper_apply_count == 1U);
    assert(disable_count[UPPER_MOTOR_ARM_J4310] == 1U);
    assert(disable_count[UPPER_MOTOR_ARM_M3508_1] == 1U);
    assert(disable_count[UPPER_MOTOR_ARM_M3508_2] == 1U);
    assert(disable_count[UPPER_MOTOR_CONVEYOR_M2006] == 0U);
    assert(disable_count[UPPER_MOTOR_GRIPPER_M2006] == 0U);
    assert(process_count == 1U);
    assert(stop_all_count == 0U);
}

static void Test_ConveyorApplyFailureIsIsolated(void)
{
    upper_robot_t robot;

    Test_Reset();
    conveyor_apply_result = false;
    robot = Test_CreateRunningRobot();
    UpperRobot_Control1ms(&robot, 2U);

    assert(robot.state == ROBOT_RUN);
    assert(robot.control_count == 1U);
    assert(arm_apply_count == 1U);
    assert(conveyor_apply_count == 1U);
    assert(gripper_apply_count == 1U);
    assert(disable_count[UPPER_MOTOR_ARM_J4310] == 0U);
    assert(disable_count[UPPER_MOTOR_ARM_M3508_1] == 0U);
    assert(disable_count[UPPER_MOTOR_ARM_M3508_2] == 0U);
    assert(disable_count[UPPER_MOTOR_CONVEYOR_M2006] == 1U);
    assert(disable_count[UPPER_MOTOR_GRIPPER_M2006] == 0U);
    assert(process_count == 1U);
    assert(stop_all_count == 0U);
}

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
    assert(conveyor_apply_count == 1U);
    assert(gripper_apply_count == 0U);
    assert(disable_count[UPPER_MOTOR_ARM_J4310] == 0U);
    assert(disable_count[UPPER_MOTOR_ARM_M3508_1] == 0U);
    assert(disable_count[UPPER_MOTOR_ARM_M3508_2] == 0U);
    assert(disable_count[UPPER_MOTOR_CONVEYOR_M2006] == 0U);
    assert(disable_count[UPPER_MOTOR_GRIPPER_M2006] == 1U);
    assert(process_count == 1U);
    assert(stop_all_count == 0U);
}

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
    assert(!robot.target.conveyor.enabled);
    assert(!robot.target.gripper.enabled);

    UpperRobot_Control1ms(&robot, 4U);
    assert(process_count == 0U);

    fresh_target = robot.target;
    fresh_target.conveyor.enabled = true;
    UpperRobot_SetTarget(&robot, &fresh_target);
    UpperRobot_Start(&robot);
    assert(robot.state == ROBOT_RUN);
    UpperRobot_Control1ms(&robot, 5U);
    assert(process_count == 1U);
}

int main(void)
{
    Test_StartupTargetsHoldDjiAtZero();
    Test_ArmCalcFailureIsIsolated();
    Test_ConveyorApplyFailureIsIsolated();
    Test_GripperCalcFailureIsIsolated();
    Test_EStopStopsAndAcceptsFreshStart();
    puts("upper robot isolation tests passed");
    return 0;
}
