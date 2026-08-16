#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "arm.h"
#include "conveyor.h"
#include "gripper.h"
#include "m2006.h"
#include "m3508.h"
#include "upper_config.h"

bool J4310_SetTorqueLimit(uint8_t motor_id, float torque_limit_nm)
{
    (void)motor_id;
    (void)torque_limit_nm;
    return true;
}

bool M3508_SetSpeedPid(uint8_t can_bus,
                       uint8_t motor_id,
                       const m3508_pid_cfg_t *cfg)
{
    (void)can_bus;
    (void)motor_id;
    (void)cfg;
    return true;
}

bool M3508_SetPositionPid(uint8_t can_bus,
                          uint8_t motor_id,
                          const m3508_pid_cfg_t *cfg)
{
    (void)can_bus;
    (void)motor_id;
    (void)cfg;
    return true;
}

bool M2006_SetSpeedPid(uint8_t can_bus,
                       uint8_t motor_id,
                       const m2006_pid_cfg_t *cfg)
{
    (void)can_bus;
    (void)motor_id;
    (void)cfg;
    return true;
}

bool M2006_SetPositionPid(uint8_t can_bus,
                          uint8_t motor_id,
                          const m2006_pid_cfg_t *cfg)
{
    (void)can_bus;
    (void)motor_id;
    (void)cfg;
    return true;
}

bool MotorManager_SetCmd(motor_manager_t *manager,
                         size_t motor_index,
                         const motor_cmd_t *cmd)
{
    (void)manager;
    (void)motor_index;
    (void)cmd;
    return true;
}

bool MotorManager_SetEnabled(motor_manager_t *manager,
                             size_t motor_index,
                             bool enabled)
{
    (void)manager;
    (void)motor_index;
    (void)enabled;
    return true;
}

static void Test_DisabledTargetsAreIgnored(void)
{
    arm_target_t arm = {0};
    arm_output_t arm_output;
    conveyor_target_t conveyor = {0};
    conveyor_output_t conveyor_output;
    gripper_target_t gripper = {0};
    gripper_output_t gripper_output;
    uint32_t index;

    arm.position_mode = true;
    arm.pid_update = true;
    arm.grip_pos_rad = NAN;
    arm.grip_vel_rad_s = NAN;
    arm.m3508_pos_rad[0] = NAN;
    arm.m3508_pos_rad[1] = NAN;
    assert(Arm_Calc(&arm, &arm_output));
    assert(arm_output.j4310.mode == MOTOR_CMD_STOP);
    for (index = 0U; index < UPPER_ARM_M3508_COUNT; index++)
    {
        assert(arm_output.m3508[index].mode == MOTOR_CMD_STOP);
    }

    conveyor.position_mode = true;
    conveyor.pid_update = true;
    conveyor.m2006_pos_rad = NAN;
    assert(Conveyor_Calc(&conveyor, &conveyor_output));
    assert(conveyor_output.m2006.mode == MOTOR_CMD_STOP);

    gripper.position_mode = true;
    gripper.pid_update = true;
    gripper.m2006_pos_rad = NAN;
    assert(Gripper_Calc(&gripper, &gripper_output));
    assert(gripper_output.m2006.mode == MOTOR_CMD_STOP);
}

static void Test_M2006PositionBoundary(void)
{
    conveyor_target_t conveyor = {0};
    conveyor_output_t conveyor_output;
    gripper_target_t gripper = {0};
    gripper_output_t gripper_output;
    float below_lower_limit;

    assert(fabsf(UPPER_M2006_POSITION_LIMIT_RAD -
                 6.28318530718f) < 0.000001f);
    assert(fabsf(UPPER_GRIPPER_M2006_POSITION_LIMIT_RAD -
                 12.56637061436f) < 0.000001f);

    below_lower_limit = nextafterf(-UPPER_M2006_POSITION_LIMIT_RAD,
                                   -INFINITY);
    conveyor.enabled = true;
    conveyor.position_mode = true;
    conveyor.m2006_pos_rad = below_lower_limit;
    assert(Conveyor_Calc(&conveyor, &conveyor_output));
    assert(conveyor_output.m2006.mode == MOTOR_CMD_POSITION);
    assert(conveyor_output.m2006.pos_rad ==
           -UPPER_M2006_POSITION_LIMIT_RAD);

    below_lower_limit = nextafterf(
        -UPPER_GRIPPER_M2006_POSITION_LIMIT_RAD, -INFINITY);
    gripper.enabled = true;
    gripper.position_mode = true;
    gripper.m2006_pos_rad = below_lower_limit;
    assert(Gripper_Calc(&gripper, &gripper_output));
    assert(gripper_output.m2006.mode == MOTOR_CMD_POSITION);
    assert(gripper_output.m2006.pos_rad ==
           -UPPER_GRIPPER_M2006_POSITION_LIMIT_RAD);

    conveyor.m2006_pos_rad = UPPER_M2006_POSITION_LIMIT_RAD + 0.01f;
    assert(!Conveyor_Calc(&conveyor, &conveyor_output));
    gripper.m2006_pos_rad =
        -UPPER_GRIPPER_M2006_POSITION_LIMIT_RAD - 0.01f;
    assert(!Gripper_Calc(&gripper, &gripper_output));
}

static void Test_RemoteActionTargets(void)
{
    assert(UPPER_REMOTE_PD13_RESET_DELAY_MS == 1000U);
    assert(UPPER_REMOTE_PD13_FIRST_M3508_DEG == 500.0f);
    assert(UPPER_REMOTE_PD13_FIRST_J4310_DEG == 90.0f);
    assert(UPPER_REMOTE_PD13_SECOND_M3508_DEG == 0.0f);
    assert(UPPER_REMOTE_PD13_SECOND_J4310_DEG == -20.0f);
    assert(UPPER_REMOTE_PD12_FIRST_M3508_DEG == 1000.0f);
    assert(UPPER_REMOTE_PD12_FIRST_J4310_DEG == 90.0f);
    assert(UPPER_REMOTE_PD12_SECOND_M3508_DEG == 0.0f);
    assert(UPPER_REMOTE_PD12_SECOND_J4310_DEG == 180.0f);
    assert(UPPER_REMOTE_PD11_FIRST_M3508_DEG == 0.0f);
    assert(UPPER_REMOTE_PD11_FIRST_J4310_DEG == 90.0f);
    assert(UPPER_REMOTE_PD11_SECOND_M3508_DEG == 0.0f);
    assert(UPPER_REMOTE_PD11_SECOND_J4310_DEG == 180.0f);
    assert(UPPER_REMOTE_PD8_FIRST_M3508_DEG == 0.0f);
    assert(UPPER_REMOTE_PD8_FIRST_J4310_DEG == 30.0f);
    assert(UPPER_REMOTE_PD8_FIRST_DELAY_MS == 1000U);
    assert(UPPER_REMOTE_PD8_SECOND_M3508_DEG == 700.0f);
    assert(UPPER_REMOTE_PD8_SECOND_J4310_DEG == 90.0f);
    assert(UPPER_REMOTE_PD8_THIRD_M3508_DEG == 0.0f);
    assert(UPPER_REMOTE_PD8_THIRD_J4310_DEG == 180.0f);
    assert(UPPER_REMOTE_PD9_FIRST_GATE_DEG == 180.0f);
    assert(UPPER_REMOTE_PD9_SECOND_GATE_DEG == 62.0f);
    assert(UPPER_REMOTE_PD10_FIRST_GRIPPER_DEG == 130.0f);
    assert(UPPER_REMOTE_PD10_SECOND_GRIPPER_DEG == 55.0f);
}

int main(void)
{
    Test_DisabledTargetsAreIgnored();
    Test_M2006PositionBoundary();
    Test_RemoteActionTargets();
    puts("upper app validation tests passed");
    return 0;
}
