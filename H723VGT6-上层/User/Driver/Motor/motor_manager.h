/**
 * @file motor_manager.h
 * @brief 定义统一电机配置、命令、端口和管理器接口。
 */

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
    MOTOR_CMD_GLOBAL_STOP,
    MOTOR_CMD_CURRENT,
    MOTOR_CMD_VELOCITY,
    MOTOR_CMD_POSITION,
    MOTOR_CMD_POSITION_VELOCITY,
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
    motor_cmd_t override_cmd[MOTOR_MANAGER_MAX_COUNT];
    bool override_enabled[MOTOR_MANAGER_MAX_COUNT];
    motor_send_t send;
    void *send_user_data;
    uint32_t sent_count;
    uint32_t send_fail_count;
    uint32_t protocol_block_count;
} motor_manager_t;

/* 功能：初始化电机管理器；用途：绑定电机配置、发送回调和用户上下文；返回 true 表示初始化成功。 */
bool MotorManager_Init(motor_manager_t *manager,
                       const motor_cfg_t *cfg,
                       size_t motor_count,
                       motor_send_t send,
                       void *user_data);
/* 功能：保存指定电机的最新控制命令；用途：为周期调度暂存目标；返回 true 表示索引和参数有效。 */
bool MotorManager_SetCmd(motor_manager_t *manager,
                         size_t motor_index,
                         const motor_cmd_t *cmd);
/* 功能：设置指定电机的使能状态；用途：控制其是否参与周期发送，并在关闭时立即发送停止命令；返回 true 表示设置成功。 */
bool MotorManager_SetEnabled(motor_manager_t *manager,
                             size_t motor_index,
                             bool enabled);
/* 功能：为指定电机设置临时覆盖命令；用途：允许调试或特殊流程绕过常规目标；返回 true 表示覆盖已生效。 */
bool MotorManager_SetOverride(motor_manager_t *manager,
                              size_t motor_index,
                              const motor_cmd_t *cmd);
/* 功能：清除指定电机的临时覆盖命令；用途：恢复常规应用目标控制；返回 true 表示覆盖已清除。 */
bool MotorManager_ClearOverride(motor_manager_t *manager,
                                size_t motor_index);
/* 功能：按周期和相位调度所有已使能电机；用途：在控制循环中发送到期命令并统计结果；无返回值表示结果记录在计数器中。 */
void MotorManager_Process(motor_manager_t *manager, uint32_t tick_ms);
/* 功能：向所有可用电机发送全局停止并清除使能；用途：正常停机或急停；无返回值表示发送结果写入统计计数。 */
void MotorManager_StopAll(motor_manager_t *manager);

#ifdef __cplusplus
}
#endif

#endif
