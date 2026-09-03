/**
 * @file motor_manager.h
 * @brief 定义统一电机配置、命令、端口和管理器接口。
 */

#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H /**< 防止 motor_manager.h 被重复包含。 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MOTOR_MANAGER_MAX_COUNT 16U /**< 静态存储空间允许管理的最大电机数量。 */

/** 标识统一电机端口支持的具体电机型号。 */
typedef enum
{
    MOTOR_MODEL_J4310, /**< 达妙 J4310 关节电机。 */
    MOTOR_MODEL_M3508, /**< DJI M3508 电机。 */
    MOTOR_MODEL_M2006, /**< DJI M2006 电机。 */
    MOTOR_MODEL_U12, /**< 宇树 U12 关节电机。 */
    MOTOR_MODEL_RS00 /**< RS00 关节电机。 */
} motor_model_t;

/** 选择统一电机命令采用的控制方式。 */
typedef enum
{
    MOTOR_CMD_STOP, /**< 停止输出。 */
    MOTOR_CMD_GLOBAL_STOP, /**< 触发整机停止。 */
    MOTOR_CMD_CURRENT, /**< 按目标电流控制。 */
    MOTOR_CMD_VELOCITY, /**< 按目标速度闭环控制。 */
    MOTOR_CMD_POSITION, /**< 按目标位置闭环控制。 */
    MOTOR_CMD_POSITION_VELOCITY, /**< 按位置和速度联合控制。 */
    MOTOR_CMD_MIT /**< 使用 MIT 位置、速度和转矩联合控制。 */
} motor_cmd_mode_t;

/** 保存 电机 运行过程中需要集中管理的数据。 */
typedef struct
{
    motor_cmd_mode_t mode; /**< 当前采用的电机控制或调试工作模式。 */
    float pos_rad; /**< 电机命令要求的目标位置，单位：弧度。 */
    float vel_rad_s; /**< 电机命令要求的目标速度，单位：弧度每秒。 */
    float torque_nm; /**< MIT 电机命令要求的前馈转矩，单位：牛米。 */
    float current_a; /**< DJI 电机命令要求的目标电流，单位：安培。 */
    float kp; /**< 比例增益。 */
    float kd; /**< 微分增益。 */
} motor_cmd_t;

/** 保存 电机 初始化和控制所需的配置参数。 */
typedef struct
{
    const char *name; /**< 用于诊断和日志显示的电机名称。 */
    motor_model_t model; /**< 电机型号。 */
    uint8_t can_bus; /**< 电机配置指定的 CAN 总线编号。 */
    uint8_t node_id; /**< 电机协议节点编号。 */
    uint16_t period_ms; /**< 该电机命令的发送周期，单位：毫秒。 */
    uint16_t phase_ms; /**< 该电机在调度周期内的发送相位，单位：毫秒。 */
    bool protocol_ready; /**< 电机协议是否已经完成启动准备。 */
} motor_cfg_t;

typedef bool (*motor_send_t)(const motor_cfg_t *cfg /**< 当前电机的型号、总线及节点配置 */,
                             const motor_cmd_t *cmd /**< 电机管理器本周期下发的控制命令 */,
                             void *user_data /**< 调用回调函数时传递的用户上下文 */);

/** 保存 电机 运行过程中需要集中管理的数据。 */
typedef struct
{
    const motor_cfg_t *cfg; /**< 电机管理器绑定的拓扑配置表。 */
    size_t motor_count; /**< 电机管理器绑定的电机数量。 */
    motor_cmd_t cmd[MOTOR_MANAGER_MAX_COUNT]; /**< 各电机在正常控制路径中等待发送的命令。 */
    bool enabled[MOTOR_MANAGER_MAX_COUNT]; /**< 各电机是否允许执行普通控制命令。 */
    motor_cmd_t override_cmd[MOTOR_MANAGER_MAX_COUNT]; /**< 各电机由安全或维护流程注入的覆盖命令。 */
    bool override_enabled[MOTOR_MANAGER_MAX_COUNT]; /**< 各电机是否优先使用覆盖命令。 */
    motor_send_t send; /**< 周期调度时用于下发电机命令的回调函数。 */
    void *send_user_data; /**< 调用回调函数时原样传回的用户上下文。 */
    uint32_t sent_count; /**< 电机管理器累计成功发送的命令数量。 */
    uint32_t send_fail_count; /**< 电机管理器累计发送失败的命令数量。 */
    uint32_t protocol_block_count; /**< 因电机协议尚未就绪而阻止发送的命令数量。 */
} motor_manager_t;

/* 功能：初始化电机管理器；用途：绑定电机配置、发送回调和用户上下文；返回 true 表示初始化成功。 */
bool MotorManager_Init(motor_manager_t *manager /**< 需要操作的电机管理器 */,
                       const motor_cfg_t *cfg /**< 当前电机的型号、总线及节点配置 */,
                       size_t motor_count /**< 调用方提供的电机配置数量 */,
                       motor_send_t send /**< 电机管理器用于下发电机命令的回调函数 */,
                       void *user_data /**< 调用回调函数时传递的用户上下文 */);
/* 功能：保存指定电机的最新控制命令；用途：为周期调度暂存目标；返回 true 表示索引和参数有效。 */
bool MotorManager_SetCmd(motor_manager_t *manager /**< 需要操作的电机管理器 */,
                         size_t motor_index /**< 电机在管理器配置表中的下标 */,
                         const motor_cmd_t *cmd /**< 待暂存的普通电机控制命令 */);
/* 功能：设置指定电机的使能状态；用途：控制其是否参与周期发送，并在关闭时立即发送停止命令；返回 true 表示设置成功。 */
bool MotorManager_SetEnabled(motor_manager_t *manager /**< 需要操作的电机管理器 */,
                             size_t motor_index /**< 电机在管理器配置表中的下标 */,
                             bool enabled /**< 是否启用指定电机的管理器输出 */);
/* 功能：为指定电机设置临时覆盖命令；用途：允许调试或特殊流程绕过常规目标；返回 true 表示覆盖已生效。 */
bool MotorManager_SetOverride(motor_manager_t *manager /**< 需要操作的电机管理器 */,
                              size_t motor_index /**< 电机在管理器配置表中的下标 */,
                              const motor_cmd_t *cmd /**< 待暂存的电机覆盖命令 */);
/* 功能：清除指定电机的临时覆盖命令；用途：恢复常规应用目标控制；返回 true 表示覆盖已清除。 */
bool MotorManager_ClearOverride(motor_manager_t *manager /**< 需要操作的电机管理器 */,
                                size_t motor_index /**< 电机在管理器配置表中的下标 */);
/* 功能：按周期和相位调度所有已使能电机；用途：在控制循环中发送到期命令并统计结果；无返回值表示结果记录在计数器中。 */
void MotorManager_Process(motor_manager_t *manager /**< 需要操作的电机管理器 */, uint32_t tick_ms /**< 当前系统毫秒时刻 */);
/* 功能：向所有可用电机发送全局停止并清除使能；用途：正常停机或急停；无返回值表示发送结果写入统计计数。 */
void MotorManager_StopAll(motor_manager_t *manager /**< 需要操作的电机管理器 */);

#ifdef __cplusplus
}
#endif

#endif
