#include "gate.h"

#include "upper_config.h"

/* 功能：更新闸门电机的 MIT 命令和使能状态；用途：把闸门目标直接提交给电机管理器；无返回值表示就地完成设置。 */
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
