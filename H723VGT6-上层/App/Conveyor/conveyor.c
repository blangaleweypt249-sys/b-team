#include "conveyor.h"

#include "upper_config.h"

void Conveyor_Update(motor_manager_t *manager,
                     const conveyor_target_t *target)
{
    motor_cmd_t left_cmd;
    motor_cmd_t right_cmd;

    if ((manager == NULL) || (target == NULL))
    {
        return;
    }

    left_cmd = (motor_cmd_t)
    {
        .mode = MOTOR_CMD_VELOCITY,
        .vel_rad_s = target->vel_rad_s[0]
    };
    right_cmd = (motor_cmd_t)
    {
        .mode = MOTOR_CMD_VELOCITY,
        .vel_rad_s = target->vel_rad_s[1]
    };
    (void)MotorManager_SetCmd(manager, UPPER_MOTOR_CONVEYOR_L, &left_cmd);
    (void)MotorManager_SetCmd(manager, UPPER_MOTOR_CONVEYOR_R, &right_cmd);
    (void)MotorManager_SetEnabled(manager, UPPER_MOTOR_CONVEYOR_L,
                                  target->enabled);
    (void)MotorManager_SetEnabled(manager, UPPER_MOTOR_CONVEYOR_R,
                                  target->enabled);
}
