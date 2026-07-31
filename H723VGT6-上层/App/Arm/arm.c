#include "arm.h"

#include "upper_config.h"

void Arm_Update(motor_manager_t *manager, const arm_target_t *target)
{
    motor_cmd_t mg_cmd;
    motor_cmd_t j_cmd;

    if ((manager == NULL) || (target == NULL))
    {
        return;
    }

    mg_cmd = (motor_cmd_t)
    {
        .mode = MOTOR_CMD_POSITION,
        .pos_rad = target->joint_pos_rad,
        .vel_rad_s = target->joint_vel_rad_s
    };
    j_cmd = (motor_cmd_t)
    {
        .mode = MOTOR_CMD_MIT,
        .pos_rad = target->grip_pos_rad,
        .vel_rad_s = target->grip_vel_rad_s,
        .kp = target->grip_kp,
        .kd = target->grip_kd
    };

    (void)MotorManager_SetCmd(manager, UPPER_MOTOR_ARM_MG5010, &mg_cmd);
    (void)MotorManager_SetCmd(manager, UPPER_MOTOR_ARM_J4310, &j_cmd);
    (void)MotorManager_SetEnabled(manager, UPPER_MOTOR_ARM_MG5010,
                                  target->enabled);
    (void)MotorManager_SetEnabled(manager, UPPER_MOTOR_ARM_J4310,
                                  target->enabled);
}
