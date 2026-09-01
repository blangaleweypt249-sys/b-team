/**
 * @file gripper.h
 * @brief 定义夹爪目标、输出数据结构和控制接口。
 */

#ifndef GRIPPER_H
/** 防止 gripper.h 被重复包含。 */
#define GRIPPER_H

#include <stdbool.h>

#include "motor_manager.h"
#include "upper_pid.h"

/* 夹爪在 C610 输出后还有 2:1 的额外减速。 */
/** 夹爪机构允许接收的位置目标绝对值上限，单位：弧度。 */
#define UPPER_GRIPPER_M2006_POSITION_LIMIT_RAD  12.56637061436f
/** 比较夹爪机构位置目标是否变化时使用的容差，单位：弧度。 */
#define UPPER_GRIPPER_M2006_POSITION_EPSILON_RAD 0.000001f
/** 夹爪机构执行位置轨迹时允许的最大速度，单位：弧度每秒。 */
#define UPPER_GRIPPER_M2006_POSITION_VEL_LIMIT_RAD_S 5.235987756f

/** 描述夹爪机构在当前控制周期内需要达到的目标。 */
typedef struct
{
    bool enabled; /**< 对应控制功能是否启用。 */
    bool position_mode; /**< 目标是否采用位置控制模式。 */
    float m2006_vel_rad_s; /**< 夹爪的目标速度，单位：弧度每秒。 */
    float m2006_pos_rad; /**< 夹爪的目标位置，单位：弧度。 */
    bool pid_update; /**< 本次命令是否同时更新 PID 参数。 */
    upper_pid_cfg_t m2006_speed_pid; /**< 夹爪对应控制环的 PID 参数或运行状态。 */
    upper_pid_cfg_t m2006_position_pid; /**< 夹爪对应控制环的 PID 参数或运行状态。 */
} gripper_target_t;

/** 保存夹爪目标校验并转换后的 M2006 命令。 */
typedef struct
{
    bool enabled; /**< 对应控制功能是否启用。 */
    motor_cmd_t m2006; /**< 已经转换完成的机构 M2006 电机命令。 */
    bool pid_update; /**< 本次命令是否同时更新 PID 参数。 */
    upper_pid_cfg_t m2006_speed_pid; /**< 夹爪对应控制环的 PID 参数或运行状态。 */
    upper_pid_cfg_t m2006_position_pid; /**< 夹爪对应控制环的 PID 参数或运行状态。 */
} gripper_output_t;

/* 功能：校验夹爪目标并生成 M2006 命令；用途：把位置或速度目标转换为驱动输入；返回 true 表示转换完成。 */
bool Gripper_Calc(const gripper_target_t *target /* 本次需要应用的控制目标 */,
                  gripper_output_t *output /* 用于写出计算结果或编码数据的缓冲区 */);
/* 功能：应用夹爪命令和可选 PID 参数；用途：向电机管理器提交目标与使能状态；返回 true 表示全部设置成功。 */
bool Gripper_Apply(motor_manager_t *manager /* 需要操作的电机管理器 */,
                   const gripper_output_t *output /* 用于写出计算结果或编码数据的缓冲区 */);

#endif
