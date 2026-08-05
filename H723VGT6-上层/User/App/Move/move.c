#include "move.h"

#include "upper_config.h"

static void Move_SetMotor(motor_manager_t *manager,
                          upper_motor_id_t motor_id,
                          float vel_rad_s,
                          bool enabled)
{
    motor_cmd_t cmd;

    cmd = (motor_cmd_t)
    {
        .mode = MOTOR_CMD_VELOCITY,
        .vel_rad_s = vel_rad_s
    };
    (void)MotorManager_SetCmd(manager, motor_id, &cmd);
    (void)MotorManager_SetEnabled(manager, motor_id, enabled);
}

void Move_Update(motor_manager_t *manager, const move_target_t *target)
{
    if ((manager == NULL) || (target == NULL))
    {
        return;
    }

    Move_SetMotor(manager, UPPER_MOTOR_MOVE_M3508_L,
                  target->m3508_vel_rad_s[0], target->enabled);
    Move_SetMotor(manager, UPPER_MOTOR_MOVE_M3508_R,
                  target->m3508_vel_rad_s[1], target->enabled);
    Move_SetMotor(manager, UPPER_MOTOR_MOVE_M2006_L,
                  target->m2006_vel_rad_s[0], target->enabled);
    Move_SetMotor(manager, UPPER_MOTOR_MOVE_M2006_R,
                  target->m2006_vel_rad_s[1], target->enabled);
}
