#include "motor_manager.h"

#include <string.h>

static bool MotorManager_CheckCfg(const motor_cfg_t *cfg, size_t motor_count)
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

bool MotorManager_Init(motor_manager_t *manager,
                       const motor_cfg_t *cfg,
                       size_t motor_count,
                       motor_send_t send,
                       void *user_data)
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

bool MotorManager_SetCmd(motor_manager_t *manager,
                         size_t motor_index,
                         const motor_cmd_t *cmd)
{
    if ((manager == NULL) || (cmd == NULL) ||
        (motor_index >= manager->motor_count))
    {
        return false;
    }

    manager->cmd[motor_index] = *cmd;
    return true;
}

bool MotorManager_SetEnabled(motor_manager_t *manager,
                             size_t motor_index,
                             bool enabled)
{
    if ((manager == NULL) || (motor_index >= manager->motor_count))
    {
        return false;
    }

    manager->enabled[motor_index] = enabled;
    return true;
}

void MotorManager_Process(motor_manager_t *manager, uint32_t tick_ms)
{
    size_t index;

    if ((manager == NULL) || (manager->send == NULL))
    {
        return;
    }

    for (index = 0U; index < manager->motor_count; index++)
    {
        const motor_cfg_t *cfg;

        cfg = &manager->cfg[index];
        if (!manager->enabled[index] ||
            ((tick_ms % cfg->period_ms) != cfg->phase_ms))
        {
            continue;
        }

        if (!cfg->protocol_ready)
        {
            manager->protocol_block_count++;
            continue;
        }

        if (manager->send(cfg, &manager->cmd[index],
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

void MotorManager_StopAll(motor_manager_t *manager)
{
    size_t index;

    if (manager == NULL)
    {
        return;
    }

    for (index = 0U; index < manager->motor_count; index++)
    {
        manager->cmd[index] = (motor_cmd_t){ .mode = MOTOR_CMD_STOP };

        if (manager->enabled[index] &&
            manager->cfg[index].protocol_ready &&
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
    }
}
