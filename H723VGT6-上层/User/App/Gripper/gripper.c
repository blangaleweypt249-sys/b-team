#include "gripper.h"

#include <math.h>

#include "upper_config.h"

bool Gripper_Calc(const gripper_target_t *target,
                  gripper_output_t *output)
{
    if ((target == NULL) || (output == NULL) ||
        !isfinite(target->m2006_vel_rad_s) ||
        (target->m2006_vel_rad_s <
         -UPPER_M2006_POSITION_VEL_LIMIT_RAD_S) ||
        (target->m2006_vel_rad_s >
         UPPER_M2006_POSITION_VEL_LIMIT_RAD_S))
    {
        return false;
    }
    output->enabled = target->enabled;
    output->m2006 = (motor_cmd_t)
    {
        .mode = MOTOR_CMD_VELOCITY,
        .vel_rad_s = target->m2006_vel_rad_s
    };
    return true;
}

bool Gripper_Apply(motor_manager_t *manager,
                   const gripper_output_t *output)
{
    bool success;

    if ((manager == NULL) || (output == NULL))
    {
        return false;
    }
    success = MotorManager_SetCmd(manager,
                                  UPPER_MOTOR_GRIPPER_M2006,
                                  &output->m2006);
    success = MotorManager_SetEnabled(manager,
                                      UPPER_MOTOR_GRIPPER_M2006,
                                      output->enabled) && success;
    return success;
}
