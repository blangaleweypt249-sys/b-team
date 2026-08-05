#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MOTOR_MANAGER_MAX_COUNT 16U

typedef enum
{
    MOTOR_MODEL_J4310,
    MOTOR_MODEL_M3508,
    MOTOR_MODEL_M2006,
    MOTOR_MODEL_U12,
    MOTOR_MODEL_RS00
} motor_model_t;

typedef enum
{
    MOTOR_CMD_STOP,
    MOTOR_CMD_CURRENT,
    MOTOR_CMD_VELOCITY,
    MOTOR_CMD_POSITION,
    MOTOR_CMD_MIT
} motor_cmd_mode_t;

typedef struct
{
    motor_cmd_mode_t mode;
    float pos_rad;
    float vel_rad_s;
    float torque_nm;
    float current_a;
    float kp;
    float kd;
} motor_cmd_t;

typedef struct
{
    const char *name;
    motor_model_t model;
    uint8_t can_bus;
    uint8_t node_id;
    uint16_t period_ms;
    uint16_t phase_ms;
    bool protocol_ready;
} motor_cfg_t;

typedef bool (*motor_send_t)(const motor_cfg_t *cfg,
                             const motor_cmd_t *cmd,
                             void *user_data);

typedef struct
{
    const motor_cfg_t *cfg;
    size_t motor_count;
    motor_cmd_t cmd[MOTOR_MANAGER_MAX_COUNT];
    bool enabled[MOTOR_MANAGER_MAX_COUNT];
    motor_send_t send;
    void *send_user_data;
    uint32_t sent_count;
    uint32_t send_fail_count;
    uint32_t protocol_block_count;
} motor_manager_t;

bool MotorManager_Init(motor_manager_t *manager,
                       const motor_cfg_t *cfg,
                       size_t motor_count,
                       motor_send_t send,
                       void *user_data);
bool MotorManager_SetCmd(motor_manager_t *manager,
                         size_t motor_index,
                         const motor_cmd_t *cmd);
bool MotorManager_SetEnabled(motor_manager_t *manager,
                             size_t motor_index,
                             bool enabled);
void MotorManager_Process(motor_manager_t *manager, uint32_t tick_ms);
void MotorManager_StopAll(motor_manager_t *manager);

#ifdef __cplusplus
}
#endif

#endif
