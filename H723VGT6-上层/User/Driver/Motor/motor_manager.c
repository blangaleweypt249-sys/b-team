/**
 * @file motor_manager.c
 * @brief 实现多型号电机的统一命令管理、覆盖控制和周期调度。
 */

#include "motor_manager.h"

#include <string.h>

/* 功能：检查电机表的数量、地址和调度参数；用途：在管理器启动前验证拓扑；返回 true 表示配置可用。 */
static bool MotorManager_CheckCfg(const motor_cfg_t *cfg /**< 当前电机的型号、总线及节点配置 */, size_t motor_count /**< 调用方提供的电机配置数量 */)
{
    size_t index;

    if ((cfg == NULL) || (motor_count == 0U) ||
        (motor_count > MOTOR_MANAGER_MAX_COUNT))
    {
        return false;
    }

    for (index = 0U; index < motor_count; index++)
    {
        if ((cfg[index].name == NULL) ||
            (cfg[index].can_bus == 0U) ||
            (cfg[index].period_ms == 0U) ||
            (cfg[index].phase_ms >= cfg[index].period_ms))
        {
            return false;
        }
    }

    return true;
}

/* 功能：初始化电机管理器；用途：绑定电机配置、发送回调和用户上下文；返回 true 表示初始化成功。 */
bool MotorManager_Init(motor_manager_t *manager /**< 需要操作的电机管理器 */,
                       const motor_cfg_t *cfg /**< 当前电机的型号、总线及节点配置 */,
                       size_t motor_count /**< 调用方提供的电机配置数量 */,
                       motor_send_t send /**< 电机管理器用于下发电机命令的回调函数 */,
                       void *user_data /**< 调用回调函数时传递的用户上下文 */)
{
    if ((manager == NULL) || (send == NULL) ||
        !MotorManager_CheckCfg(cfg, motor_count))
    {
        return false;
    }

    (void)memset(manager, 0, sizeof(*manager));
    manager->cfg = cfg;
    manager->motor_count = motor_count;
    manager->send = send;
    manager->send_user_data = user_data;
    return true;
}

/* 功能：保存指定电机的最新控制命令；用途：为周期调度暂存目标；返回 true 表示索引和参数有效。 */
bool MotorManager_SetCmd(motor_manager_t *manager /**< 需要操作的电机管理器 */,
                         size_t motor_index /**< 电机在管理器配置表中的下标 */,
                         const motor_cmd_t *cmd /**< 待暂存的普通电机控制命令 */)
{
    if ((manager == NULL) || (cmd == NULL) ||
        (motor_index >= manager->motor_count))
    {
        return false;
    }

    manager->cmd[motor_index] = *cmd;
    return true;
}

/* 功能：设置指定电机的使能状态；用途：控制其是否参与周期发送，并在关闭时立即发送停止命令；返回 true 表示设置成功。 */
bool MotorManager_SetEnabled(motor_manager_t *manager /**< 需要操作的电机管理器 */,
                             size_t motor_index /**< 电机在管理器配置表中的下标 */,
                             bool enabled /**< 是否启用指定电机的管理器输出 */)
{
    motor_cmd_t stop_cmd;

    if ((manager == NULL) || (motor_index >= manager->motor_count))
    {
        return false;
    }

    if (manager->enabled[motor_index] && !enabled)
    {
        stop_cmd = (motor_cmd_t){ .mode = MOTOR_CMD_STOP };
        manager->cmd[motor_index] = stop_cmd;
        if (manager->cfg[motor_index].protocol_ready &&
            (manager->send != NULL))
        {
            if (manager->send(&manager->cfg[motor_index], &stop_cmd,
                              manager->send_user_data))
            {
                manager->sent_count++;
            }
            else
            {
                manager->send_fail_count++;
            }
        }
    }

    manager->enabled[motor_index] = enabled;
    return true;
}

/* 功能：为指定电机设置临时覆盖命令；用途：允许调试或特殊流程绕过常规目标；返回 true 表示覆盖已生效。 */
bool MotorManager_SetOverride(motor_manager_t *manager /**< 需要操作的电机管理器 */,
                              size_t motor_index /**< 电机在管理器配置表中的下标 */,
                              const motor_cmd_t *cmd /**< 待暂存的电机覆盖命令 */)
{
    if ((manager == NULL) || (cmd == NULL) ||
        (motor_index >= manager->motor_count))
    {
        return false;
    }
    manager->override_cmd[motor_index] = *cmd;
    manager->override_enabled[motor_index] = true;
    return true;
}

/* 功能：清除指定电机的临时覆盖命令；用途：恢复常规应用目标控制；返回 true 表示覆盖已清除。 */
bool MotorManager_ClearOverride(motor_manager_t *manager /**< 需要操作的电机管理器 */,
                                size_t motor_index /**< 电机在管理器配置表中的下标 */)
{
    motor_cmd_t stop_cmd;

    if ((manager == NULL) || (motor_index >= manager->motor_count))
    {
        return false;
    }
    if (manager->override_enabled[motor_index])
    {
        stop_cmd = (motor_cmd_t){ .mode = MOTOR_CMD_STOP };
        if (manager->cfg[motor_index].protocol_ready &&
            (manager->send != NULL))
        {
            if (manager->send(&manager->cfg[motor_index], &stop_cmd,
                              manager->send_user_data))
            {
                manager->sent_count++;
            }
            else
            {
                manager->send_fail_count++;
            }
        }
    }
    manager->override_enabled[motor_index] = false;
    manager->override_cmd[motor_index] =
        (motor_cmd_t){ .mode = MOTOR_CMD_STOP };
    return true;
}

/* 功能：按周期和相位调度所有已使能电机；用途：在控制循环中发送到期命令并统计结果；无返回值表示结果记录在计数器中。 */
void MotorManager_Process(motor_manager_t *manager /**< 需要操作的电机管理器 */, uint32_t tick_ms /**< 当前系统毫秒时刻 */)
{
    size_t index;

    if ((manager == NULL) || (manager->send == NULL))
    {
        return;
    }

    for (index = 0U; index < manager->motor_count; index++)
    {
        const motor_cfg_t *cfg;
        const motor_cmd_t *command;

        cfg = &manager->cfg[index];
        if ((!manager->enabled[index] &&
             !manager->override_enabled[index]) ||
            ((tick_ms % cfg->period_ms) != cfg->phase_ms))
        {
            continue;
        }

        if (!cfg->protocol_ready)
        {
            manager->protocol_block_count++;
            continue;
        }

        command = manager->override_enabled[index] ?
                  &manager->override_cmd[index] : &manager->cmd[index];
        if (manager->send(cfg, command,
                          manager->send_user_data))
        {
            manager->sent_count++;
        }
        else
        {
            manager->send_fail_count++;
        }
    }
}

/* 功能：向所有可用电机发送全局停止并清除使能；用途：正常停机或急停；无返回值表示发送结果写入统计计数。 */
void MotorManager_StopAll(motor_manager_t *manager /**< 需要操作的电机管理器 */)
{
    size_t index;

    if (manager == NULL)
    {
        return;
    }

    for (index = 0U; index < manager->motor_count; index++)
    {
        manager->cmd[index] =
            (motor_cmd_t){ .mode = MOTOR_CMD_GLOBAL_STOP };

        if (manager->cfg[index].protocol_ready &&
            (manager->send != NULL))
        {
            if (manager->send(&manager->cfg[index], &manager->cmd[index],
                              manager->send_user_data))
            {
                manager->sent_count++;
            }
            else
            {
                manager->send_fail_count++;
            }
        }

        manager->enabled[index] = false;
        manager->override_enabled[index] = false;
        manager->override_cmd[index] =
            (motor_cmd_t){ .mode = MOTOR_CMD_STOP };
    }
}
