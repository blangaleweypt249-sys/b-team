/**
 * @file arm.h
 * @brief 定义机械臂目标、输出数据结构和控制接口。
 */

#ifndef ARM_H
/** 防止 arm.h 被重复包含。 */
#define ARM_H

#include <stdbool.h>

#include "upper_motor_port.h"
#include "upper_pid.h"

/** 机械臂机构中参与控制的 M3508 电机数量。 */
#define UPPER_ARM_M3508_COUNT                  2U

/* 机械臂命令边界；MIT 映射限值必须与 J4310 持久配置一致。 */
/** 机械臂 J4310 关节命令允许使用的位置映射满量程，单位：弧度。 */
#define UPPER_J4310_POSITION_MAX_RAD           12.5f
/** 机械臂 J4310 关节 MIT 协议速度字段映射的满量程，单位：弧度每秒。 */
#define UPPER_J4310_VELOCITY_MAX_RAD_S         30.0f
/** 机械臂 J4310 关节 MIT 协议转矩字段映射的满量程，单位：牛米。 */
#define UPPER_J4310_TORQUE_MAP_MAX_NM          10.0f
/** 机械臂 J4310 关节控制命令允许使用的最大比例增益。 */
#define UPPER_J4310_KP_MAX                     49.0f
/** 机械臂 J4310 关节控制命令允许使用的最大微分增益。 */
#define UPPER_J4310_KD_MAX                      0.95f

/** 机械臂 M3508 输出轴允许接收的最小位置目标，单位：弧度。 */
#define UPPER_M3508_POSITION_MIN_RAD            0.0f
/** 机械臂 M3508 输出轴允许接收的最大位置目标，单位：弧度。 */
#define UPPER_M3508_POSITION_MAX_RAD            20.9439510239f
/** 机械臂 M3508 输出轴执行位置轨迹时允许的最大速度，单位：弧度每秒。 */
#define UPPER_M3508_POSITION_VEL_LIMIT_RAD_S    15.708f

/** 描述机械臂在当前控制周期内需要达到的目标。 */
typedef struct
{
    bool enabled; /**< 对应控制功能是否启用。 */
    bool j4310_commanded; /**< 本周期是否需要向机械臂 J4310 下发控制命令。 */
    bool m3508_enabled; /**< 机械臂两台 M3508 是否允许输出。 */
    bool position_mode; /**< 目标是否采用位置控制模式。 */
    float grip_pos_rad; /**< 机械臂的目标位置，单位：弧度。 */
    float grip_vel_rad_s; /**< 机械臂的目标速度，单位：弧度每秒。 */
    float grip_kp; /**< 夹持关节 MIT 位置项的比例增益。 */
    float grip_kd; /**< 夹持关节 MIT 速度项的微分增益。 */
    float grip_torque_nm; /**< 夹持关节 MIT 命令要求的前馈转矩，单位：牛米。 */
    float grip_torque_limit_nm; /**< 机械臂的转矩上限，单位：牛米。 */
    float m3508_vel_rad_s[UPPER_ARM_M3508_COUNT]; /**< 机械臂的目标速度，单位：弧度每秒。 */
    float m3508_pos_rad[UPPER_ARM_M3508_COUNT]; /**< 机械臂的目标位置，单位：弧度。 */
    bool pid_update; /**< 本次命令是否同时更新 PID 参数。 */
    upper_pid_cfg_t m3508_speed_pid; /**< 机械臂对应控制环的 PID 参数或运行状态。 */
    upper_pid_cfg_t m3508_position_pid; /**< 机械臂对应控制环的 PID 参数或运行状态。 */
} arm_target_t;

/** 保存机械臂目标校验并转换后的电机命令。 */
typedef struct
{
    bool enabled; /**< 对应控制功能是否启用。 */
    bool m3508_enabled; /**< 机械臂两台 M3508 是否允许输出。 */
    motor_cmd_t j4310; /**< 已经转换完成的机械臂 J4310 电机命令。 */
    motor_cmd_t m3508[UPPER_ARM_M3508_COUNT]; /**< 已经转换完成的机械臂 M3508 电机命令。 */
    bool pid_update; /**< 本次命令是否同时更新 PID 参数。 */
    float j4310_torque_limit_nm; /**< 机械臂的转矩上限，单位：牛米。 */
    upper_pid_cfg_t m3508_speed_pid; /**< 机械臂对应控制环的 PID 参数或运行状态。 */
    upper_pid_cfg_t m3508_position_pid; /**< 机械臂对应控制环的 PID 参数或运行状态。 */
} arm_output_t;

/* 功能：校验机械臂目标并转换为各电机命令；用途：生成单周期可统一下发的输出快照；返回 true 表示目标合法且转换完成。 */
bool Arm_Calc(const arm_target_t *target /* 本次需要应用的控制目标 */, arm_output_t *output /* 用于写出计算结果或编码数据的缓冲区 */);
/* 功能：应用机械臂输出及可选 PID 更新；用途：把 J4310 和 M3508 命令提交给电机管理器；返回 true 表示全部设置成功。 */
bool Arm_Apply(motor_manager_t *manager /* 需要操作的电机管理器 */, const arm_output_t *output /* 用于写出计算结果或编码数据的缓冲区 */);

#endif
