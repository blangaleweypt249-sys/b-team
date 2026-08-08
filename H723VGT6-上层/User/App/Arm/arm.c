#include "arm.h"

#include <math.h>

#include "upper_config.h"

static bool Arm_ValueWithin(float value, float limit)
{
    return isfinite(value) && (value >= -limit) && (value <= limit);
}

bool Arm_Calc(const arm_target_t *target, arm_output_t *output)
{
    uint32_t index;

    if ((target == NULL) || (output == NULL))
    {
        return false;
    }
    if (!Arm_ValueWithin(target->grip_pos_rad,
                         UPPER_J4310_POSITION_MAX_RAD) ||
        !Arm_ValueWithin(target->grip_vel_rad_s,
                         UPPER_J4310_VELOCITY_MAX_RAD_S) ||
        !isfinite(target->grip_kp) || (target->grip_kp < 0.0f) ||
        (target->grip_kp > UPPER_J4310_KP_MAX) ||
        !isfinite(target->grip_kd) || (target->grip_kd < 0.0f) ||
        (target->grip_kd > UPPER_J4310_KD_MAX))
    {
        return false;
    }
    for (index = 0U; index < UPPER_ARM_M3508_COUNT; index++)
    {
        if (!Arm_ValueWithin(target->m3508_vel_rad_s[index],
                             UPPER_M3508_POSITION_VEL_LIMIT_RAD_S))
        {
            return false;
        }
    }

    output->enabled = target->enabled;
    output->j4310 = (motor_cmd_t)
    {
        .mode = MOTOR_CMD_MIT,
        .pos_rad = target->grip_pos_rad,
        .vel_rad_s = target->grip_vel_rad_s,
        .kp = target->grip_kp,
        .kd = target->grip_kd
    };
    for (index = 0U; index < UPPER_ARM_M3508_COUNT; index++)
    {
        output->m3508[index] = (motor_cmd_t)
        {
            .mode = MOTOR_CMD_VELOCITY,
            .vel_rad_s = target->m3508_vel_rad_s[index]
        };
    }
    return true;
}

bool Arm_Apply(motor_manager_t *manager, const arm_output_t *output)
{
    uint32_t index;
    bool success;

    if ((manager == NULL) || (output == NULL))
    {
        return false;
    }

    success = true;
    for (index = 0U; index < UPPER_ARM_M3508_COUNT; index++)
    {
        success = MotorManager_SetCmd(
                      manager,
                      (size_t)UPPER_MOTOR_ARM_M3508_1 + index,
                      &output->m3508[index]) && success;
        success = MotorManager_SetEnabled(
                      manager,
                      (size_t)UPPER_MOTOR_ARM_M3508_1 + index,
                      output->enabled) && success;
    }
    success = MotorManager_SetCmd(manager,
                                  UPPER_MOTOR_ARM_J4310,
                                  &output->j4310) && success;
    success = MotorManager_SetEnabled(manager,
                                      UPPER_MOTOR_ARM_J4310,
                                      output->enabled) && success;
    return success;
}
