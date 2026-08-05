#include "gate.h"

#include "upper_config.h"

void Gate_Update(motor_manager_t *manager, const gate_target_t *target)
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
    (void)MotorManager_SetCmd(manager, UPPER_MOTOR_GATE_M2006, &cmd);
    (void)MotorManager_SetEnabled(manager, UPPER_MOTOR_GATE_M2006,
                                  target->enabled);
}
