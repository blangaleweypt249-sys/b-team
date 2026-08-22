/**
 * @file test_upper_app_validation.c
 * @brief 验证上层应用目标校验和边界处理。
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "arm.h"
#include "gate.h"
#include "gripper.h"
#include "m2006.h"
#include "m3508.h"
#include "upper_entry.h"

/* 功能：提供 J4310_SetTorqueLimit 的测试桩实现；用途：隔离外部依赖并记录或模拟调用结果。 */
bool J4310_SetTorqueLimit(uint8_t motor_id, float torque_limit_nm)
{
    (void)motor_id;
    (void)torque_limit_nm;
    return true;
}

/* 功能：提供 M3508_SetSpeedPid 的测试桩实现；用途：隔离外部依赖并记录或模拟调用结果。 */
bool M3508_SetSpeedPid(uint8_t can_bus,
                       uint8_t motor_id,
                       const m3508_pid_cfg_t *cfg)
{
    (void)can_bus;
    (void)motor_id;
    (void)cfg;
    return true;
}

/* 功能：提供 M3508_SetPositionPid 的测试桩实现；用途：隔离外部依赖并记录或模拟调用结果。 */
bool M3508_SetPositionPid(uint8_t can_bus,
                          uint8_t motor_id,
                          const m3508_pid_cfg_t *cfg)
{
    (void)can_bus;
    (void)motor_id;
    (void)cfg;
    return true;
}

/* 功能：提供 M2006_SetSpeedPid 的测试桩实现；用途：隔离外部依赖并记录或模拟调用结果。 */
bool M2006_SetSpeedPid(uint8_t can_bus,
                       uint8_t motor_id,
                       const m2006_pid_cfg_t *cfg)
{
    (void)can_bus;
    (void)motor_id;
    (void)cfg;
    return true;
}

/* 功能：提供 M2006_SetPositionPid 的测试桩实现；用途：隔离外部依赖并记录或模拟调用结果。 */
bool M2006_SetPositionPid(uint8_t can_bus,
                          uint8_t motor_id,
                          const m2006_pid_cfg_t *cfg)
{
    (void)can_bus;
    (void)motor_id;
    (void)cfg;
    return true;
}

/* 功能：提供 MotorManager_SetCmd 的测试桩实现；用途：隔离外部依赖并记录或模拟调用结果。 */
bool MotorManager_SetCmd(motor_manager_t *manager,
                         size_t motor_index,
                         const motor_cmd_t *cmd)
{
    (void)manager;
    (void)motor_index;
    (void)cmd;
    return true;
}

/* 功能：提供 MotorManager_SetEnabled 的测试桩实现；用途：隔离外部依赖并记录或模拟调用结果。 */
bool MotorManager_SetEnabled(motor_manager_t *manager,
                             size_t motor_index,
                             bool enabled)
{
    (void)manager;
    (void)motor_index;
    (void)enabled;
    return true;
}

/* 功能：执行 DisabledTargetsAreIgnored 场景测试；用途：验证对应输入下的行为和断言；无返回值表示由断言报告结果。 */
static void Test_DisabledTargetsAreIgnored(void)
{
    arm_target_t arm = {0};
    arm_output_t arm_output;
    gate_target_t gate = {0};
    gate_output_t gate_output;
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

    gate.position_mode = true;
    gate.pid_update = true;
    gate.m2006_pos_rad = NAN;
    assert(Gate_Calc(&gate, &gate_output));
    assert(gate_output.m2006.mode == MOTOR_CMD_STOP);

    gripper.position_mode = true;
    gripper.pid_update = true;
    gripper.m2006_pos_rad = NAN;
    assert(Gripper_Calc(&gripper, &gripper_output));
    assert(gripper_output.m2006.mode == MOTOR_CMD_STOP);
}

/* 功能：执行 M2006PositionBoundary 场景测试；用途：验证对应输入下的行为和断言；无返回值表示由断言报告结果。 */
static void Test_M2006PositionBoundary(void)
{
    gate_target_t gate = {0};
    gate_output_t gate_output;
    gripper_target_t gripper = {0};
    gripper_output_t gripper_output;
    float below_lower_limit;

    assert(fabsf(UPPER_GATE_M2006_POSITION_LIMIT_RAD -
                 6.28318530718f) < 0.000001f);
    assert(fabsf(UPPER_GRIPPER_M2006_POSITION_LIMIT_RAD -
                 12.56637061436f) < 0.000001f);

    below_lower_limit = nextafterf(-UPPER_GATE_M2006_POSITION_LIMIT_RAD,
                                   -INFINITY);
    gate.enabled = true;
    gate.position_mode = true;
    gate.m2006_pos_rad = below_lower_limit;
    assert(Gate_Calc(&gate, &gate_output));
    assert(gate_output.m2006.mode == MOTOR_CMD_POSITION);
    assert(gate_output.m2006.pos_rad ==
           -UPPER_GATE_M2006_POSITION_LIMIT_RAD);

    below_lower_limit = nextafterf(
        -UPPER_GRIPPER_M2006_POSITION_LIMIT_RAD, -INFINITY);
    gripper.enabled = true;
    gripper.position_mode = true;
    gripper.m2006_pos_rad = below_lower_limit;
    assert(Gripper_Calc(&gripper, &gripper_output));
    assert(gripper_output.m2006.mode == MOTOR_CMD_POSITION);
    assert(gripper_output.m2006.pos_rad ==
           -UPPER_GRIPPER_M2006_POSITION_LIMIT_RAD);

    gate.m2006_pos_rad = UPPER_GATE_M2006_POSITION_LIMIT_RAD + 0.01f;
    assert(!Gate_Calc(&gate, &gate_output));
    gripper.m2006_pos_rad =
        -UPPER_GRIPPER_M2006_POSITION_LIMIT_RAD - 0.01f;
    assert(!Gripper_Calc(&gripper, &gripper_output));
}

/* 功能：执行 RemoteActionTargets 场景测试；用途：验证对应输入下的行为和断言；无返回值表示由断言报告结果。 */
static void Test_RemoteActionTargets(void)
{
    assert(UPPER_J4310_POSITION_KP == 30.0f);
    assert(UPPER_J4310_POSITION_KD == 0.95f);
    assert(UPPER_STALL_ARMING_GRACE_MS == 500U);
    assert(UPPER_STALL_CONFIRM_MS == 3000U);
    assert(UPPER_J4310_STALL_MIN_ERROR_DEG == 15.0f);
    assert(UPPER_J4310_STALL_MAX_VELOCITY_DEG_S == 1.0f);
    assert(UPPER_J4310_STALL_MIN_TORQUE_NM == 3.0f);
    assert(UPPER_J4310_STALL_RECOVERY_DEG == 90.0f);
    assert(UPPER_GATE_STALL_MIN_ERROR_DEG == 12.0f);
    assert(UPPER_GATE_STALL_MAX_VELOCITY_DEG_S == 1.0f);
    assert(UPPER_GATE_STALL_MIN_CURRENT_A == 3.0f);
    assert(UPPER_GATE_STALL_RECOVERY_DEG == 80.0f);
    assert(UPPER_GRIPPER_STALL_MIN_ERROR_DEG == 12.0f);
    assert(UPPER_GRIPPER_STALL_MAX_VELOCITY_DEG_S == 1.0f);
    assert(UPPER_GRIPPER_STALL_MIN_CURRENT_A == 3.0f);
    assert(UPPER_REMOTE_PD13_FIRST_M3508_DEG == 500.0f);
    assert(UPPER_REMOTE_PD13_FIRST_J4310_DEG == 90.0f);
    assert(UPPER_REMOTE_PD13_SECOND_M3508_DEG == 1050.0f);
    assert(UPPER_REMOTE_PD13_SECOND_J4310_DEG == 90.0f);
    assert(UPPER_REMOTE_AUTO_PD13_SECOND_J4310_DEG == 70.0f);
    assert(UPPER_REMOTE_FLIP_PD13_NEXT_M3508_DEG == 1000.0f);
    assert(UPPER_REMOTE_FLIP_PD13_NEXT_J4310_DEG == 90.0f);
    assert(UPPER_REMOTE_FLIP_FINAL_M3508_DEG == 500.0f);
    assert(UPPER_REMOTE_FLIP_FINAL_J4310_DEG == 90.0f);
    assert(UPPER_REMOTE_PD12_FIRST_M3508_DEG == 0.0f);
    assert(UPPER_REMOTE_PD12_FIRST_J4310_DEG == 90.0f);
    assert(UPPER_REMOTE_PD12_SECOND_M3508_DEG == 850.0f);
    assert(UPPER_REMOTE_PD12_SECOND_J4310_DEG == 90.0f);
    assert(UPPER_REMOTE_FLIP_PD12_NEXT_M3508_DEG == 850.0f);
    assert(UPPER_REMOTE_FLIP_PD12_NEXT_J4310_DEG == 90.0f);
    assert(UPPER_REMOTE_FLIP_FIRST_CLOSE_J4310_DEG == 180.0f);
    assert(UPPER_REMOTE_FLIP_FIRST_CLOSE_DELAY_MS == 200U);
    assert(UPPER_REMOTE_FLIP_FIRST_FINAL_M3508_DEG == 500.0f);
    assert(UPPER_REMOTE_FLIP_FIRST_FINAL_OPEN_DELAY_MS == 1500U);
    assert(UPPER_REMOTE_AUTO_START_GRIPPER_DEG == 55.0f);
    assert(UPPER_REMOTE_PD11_FIRST_M3508_DEG == 0.0f);
    assert(UPPER_REMOTE_PD11_FIRST_J4310_DEG == 165.0f);
    assert(UPPER_REMOTE_PD11_SECOND_M3508_DEG == 0.0f);
    assert(UPPER_REMOTE_PD11_SECOND_J4310_DEG == 240.0f);
    assert(UPPER_REMOTE_PD11_GATE_DEG == 60.0f);
    assert(UPPER_REMOTE_STORE2_PD11_GATE_DEG == 0.0f);
    assert(UPPER_REMOTE_STORE2_FINAL_J4310_DEG == 180.0f);
    assert(UPPER_REMOTE_STORE2_PD11_DOUBLE_M3508_DEG == 1050.0f);
    assert(UPPER_REMOTE_STORE2_PD11_DOUBLE_J4310_DEG == 90.0f);
    assert(UPPER_REMOTE_PD8_FIRST_M3508_DEG == 0.0f);
    assert(UPPER_REMOTE_PD8_FIRST_J4310_DEG == 40.0f);
    assert(UPPER_REMOTE_PD8_SECOND_M3508_DEG == 0.0f);
    assert(UPPER_REMOTE_PD8_SECOND_J4310_DEG == -20.0f);
    assert(UPPER_REMOTE_PD9_ZERO_GATE_DEG == 0.0f);
    assert(UPPER_REMOTE_PD9_FIRST_GATE_DEG == 180.0f);
    assert(UPPER_REMOTE_PD9_SECOND_GATE_DEG == 60.0f);
    assert(UPPER_REMOTE_PC1_GATE_DEG == 80.0f);
    assert(UPPER_REMOTE_PC1_GATE_DISABLE_MIN_DEG == 79.0f);
    assert(UPPER_REMOTE_PC1_GATE_DISABLE_MAX_DEG == 81.0f);
    assert(UPPER_REMOTE_PC0_SECOND_BRANCH_M3508_DEG == 0.0f);
    assert(UPPER_REMOTE_PC0_THIRD_BRANCH_M3508_DEG == 850.0f);
    assert(UPPER_REMOTE_PC0_FIRST_J4310_DEG == 90.0f);
    assert(UPPER_REMOTE_PC0_CLOSE_M3508_DEG == 0.0f);
    assert(UPPER_REMOTE_PC0_CLOSE_J4310_DEG == 90.0f);
    assert(UPPER_REMOTE_PC0_SECOND_J4310_DEG == -20.0f);
    assert(UPPER_REMOTE_PC0_GATE_FIRST_DEG == 180.0f);
    assert(UPPER_REMOTE_PC0_GATE_FINAL_DEG == 68.0f);
    assert(UPPER_REMOTE_PC0_FINAL_M3508_DEG == 0.0f);
    assert(UPPER_REMOTE_PC0_FINAL_J4310_DEG == 90.0f);
    assert(UPPER_REMOTE_PD10_FIRST_GRIPPER_DEG == 55.0f);
    assert(UPPER_REMOTE_PD10_SECOND_GRIPPER_DEG == 125.0f);
    assert(UPPER_REMOTE_AUTO_START_IS_AVAILABLE(false, false, false));
    assert(!UPPER_REMOTE_AUTO_START_IS_AVAILABLE(true, false, false));
    assert(!UPPER_REMOTE_AUTO_START_IS_AVAILABLE(false, true, false));
    assert(!UPPER_REMOTE_AUTO_START_IS_AVAILABLE(false, false, true));
    assert(UPPER_REMOTE_FINAL_J4310_DELAY_MS == 500U);
    assert(UPPER_REMOTE_PD12_240_HOLD_MS == 1500U);
    assert(UPPER_REMOTE_PD11_OPEN_DELAY_MS == 1200U);
    assert(UPPER_REMOTE_PD11_J4310_DELAY_MS == 500U);
    assert(UPPER_REMOTE_PD11_CLOSE_DELAY_MS == 1800U);
    assert(UPPER_REMOTE_PD11_J4310_AFTER_CLOSE_DELAY_MS == 200U);
    assert(UPPER_REMOTE_STORE2_PD11_DOUBLE_CLICK_MS == 500U);
    assert(UPPER_REMOTE_STORE2_PD11_RETURN_DELAY_MS == 1500U);
    assert(UPPER_REMOTE_PC0_PD8_DELAY_MS == 500U);
    assert(UPPER_REMOTE_PC0_FINAL_DELAY_MS == 0U);
    assert(UPPER_REMOTE_MODE_FROM_SWITCHES(
               UPPER_REMOTE_PRIMARY_SWITCH_PE0 |
               UPPER_REMOTE_PRIMARY_SWITCH_PD6) ==
           UPPER_REMOTE_MODE_STORE3_AUTO);
    assert(UPPER_REMOTE_MODE_FROM_SWITCHES(
               UPPER_REMOTE_PRIMARY_SWITCH_PD6) ==
           UPPER_REMOTE_MODE_STORE3_MANUAL);
    assert(UPPER_REMOTE_MODE_FROM_SWITCHES(
               UPPER_REMOTE_PRIMARY_SWITCH_PE0) ==
           UPPER_REMOTE_MODE_STORE2_AUTO);
    assert(UPPER_REMOTE_MODE_FROM_SWITCHES(0U) ==
           UPPER_REMOTE_MODE_STORE2_MANUAL);
    assert(UPPER_REMOTE_AUTO_HAS_240_STAGE(
               UPPER_REMOTE_MODE_STORE3_AUTO));
    assert(!UPPER_REMOTE_AUTO_HAS_240_STAGE(
               UPPER_REMOTE_MODE_STORE2_AUTO));
    assert(UPPER_REMOTE_AUTO_FINAL_J4310_DEG(
               UPPER_REMOTE_MODE_STORE3_AUTO) == 165.0f);
    assert(UPPER_REMOTE_AUTO_FINAL_J4310_DEG(
               UPPER_REMOTE_MODE_STORE2_AUTO) == 180.0f);
}

/* 功能：执行 M3508PositionBoundary 场景测试；用途：验证对应输入下的行为和断言；无返回值表示由断言报告结果。 */
static void Test_M3508PositionBoundary(void)
{
    arm_target_t arm = {0};
    arm_output_t output;

    assert(fabsf(UPPER_M3508_POSITION_MAX_RAD -
                 20.9439510239f) < 0.000001f);
    arm.m3508_enabled = true;
    arm.position_mode = true;
    arm.m3508_pos_rad[0] = UPPER_M3508_POSITION_MIN_RAD;
    arm.m3508_pos_rad[1] = UPPER_M3508_POSITION_MAX_RAD;
    assert(Arm_Calc(&arm, &output));
    assert(output.m3508[0].pos_rad == UPPER_M3508_POSITION_MIN_RAD);
    assert(output.m3508[1].pos_rad == UPPER_M3508_POSITION_MAX_RAD);

    arm.m3508_pos_rad[0] = nextafterf(
        UPPER_M3508_POSITION_MIN_RAD, -INFINITY);
    assert(!Arm_Calc(&arm, &output));
    arm.m3508_pos_rad[0] = UPPER_M3508_POSITION_MIN_RAD;
    arm.m3508_pos_rad[1] = nextafterf(
        UPPER_M3508_POSITION_MAX_RAD, INFINITY);
    assert(!Arm_Calc(&arm, &output));
}

/* 功能：运行本文件的上层应用目标校验和边界处理测试；用途：集中执行断言用例；返回 0 表示全部测试通过。 */
int main(void)
{
    Test_DisabledTargetsAreIgnored();
    Test_M2006PositionBoundary();
    Test_M3508PositionBoundary();
    Test_RemoteActionTargets();
    puts("upper app validation tests passed");
    return 0;
}
