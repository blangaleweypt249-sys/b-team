#include "ore.h"

#include "upper_config.h"

void Ore_Update(motor_manager_t *manager, const ore_target_t *target)
{
    motor_cmd_t cmd;

    if ((manager == NULL) || (target == NULL))
    {
        return;
    }

    cmd = (motor_cmd_t)
    {
        .mode = MOTOR_CMD_VELOCITY,
        .vel_rad_s = target->vel_rad_s
    };
    (void)MotorManager_SetCmd(manager, UPPER_MOTOR_ORE_M2006, &cmd);
    (void)MotorManager_SetEnabled(manager, UPPER_MOTOR_ORE_M2006,
                                  target->enabled);
}
