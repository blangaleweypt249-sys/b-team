#include <assert.h>
#include <stdbool.h>
#include <stddef.h>

#include "motor_manager.h"

static motor_cmd_t test_last_command;
static size_t test_send_count;

static bool Test_Send(const motor_cfg_t *cfg,
                      const motor_cmd_t *cmd,
                      void *user_data)
{
    (void)cfg;
    (void)user_data;
    test_last_command = *cmd;
    test_send_count++;
    return true;
}

int main(void)
{
    const motor_cfg_t cfg = {
        .name = "J4310",
        .model = MOTOR_MODEL_J4310,
        .can_bus = 1U,
        .node_id = 6U,
        .period_ms = 1U,
        .phase_ms = 0U,
        .protocol_ready = true
    };
    motor_manager_t manager;
    motor_cmd_t normal_command = {
        .mode = MOTOR_CMD_MIT,
        .pos_rad = 1.0f
    };
    motor_cmd_t override_command = {
        .mode = MOTOR_CMD_MIT,
        .pos_rad = 0.5f
    };

    assert(MotorManager_Init(&manager, &cfg, 1U, Test_Send, NULL));
    assert(MotorManager_SetCmd(&manager, 0U, &normal_command));
    assert(MotorManager_SetEnabled(&manager, 0U, true));
    assert(MotorManager_SetOverride(&manager, 0U, &override_command));

    MotorManager_Process(&manager, 0U);
    assert(test_send_count == 1U);
    assert(test_last_command.mode == MOTOR_CMD_MIT);
    assert(test_last_command.pos_rad == 0.5f);

    assert(MotorManager_ClearOverride(&manager, 0U));
    assert(test_send_count == 2U);
    assert(test_last_command.mode == MOTOR_CMD_STOP);
    MotorManager_Process(&manager, 1U);
    assert(test_send_count == 3U);
    assert(test_last_command.pos_rad == 1.0f);

    assert(MotorManager_SetEnabled(&manager, 0U, false));
    assert(MotorManager_SetOverride(&manager, 0U, &override_command));
    MotorManager_Process(&manager, 2U);
    assert(test_last_command.pos_rad == 0.5f);

    MotorManager_StopAll(&manager);
    assert(test_last_command.mode == MOTOR_CMD_GLOBAL_STOP);
    assert(!manager.enabled[0]);
    assert(!manager.override_enabled[0]);
    MotorManager_Process(&manager, 3U);
    assert(test_last_command.mode == MOTOR_CMD_GLOBAL_STOP);

    test_send_count = 0U;
    assert(MotorManager_Init(&manager, &cfg, 1U, Test_Send, NULL));
    MotorManager_StopAll(&manager);
    assert(test_send_count == 1U);
    assert(test_last_command.mode == MOTOR_CMD_GLOBAL_STOP);
    return 0;
}
