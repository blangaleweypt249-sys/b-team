/**
 * @file gripper.h
 * @brief 定义夹爪目标、输出数据结构和控制接口。
 */

#ifndef GRIPPER_H
#define GRIPPER_H /**< 防止 gripper.h 被重复包含。 */

#include <stdbool.h>

#include "motor_manager.h"
#include "upper_pid.h"

#define UPPER_GRIPPER_M2006_POSITION_LIMIT_RAD  12.56637061436f /**< 考虑 C610 后级 2:1 减速比后的夹爪位置目标绝对值上限，单位：弧度。 */
#define UPPER_GRIPPER_M2006_POSITION_EPSILON_RAD 0.000001f /**< 比较夹爪机构位置目标是否变化时使用的容差，单位：弧度。 */
#define UPPER_GRIPPER_M2006_POSITION_VEL_LIMIT_RAD_S 5.235987756f /**< 夹爪机构执行位置轨迹时允许的最大速度，单位：弧度每秒。 */

/** 描述夹爪机构在当前控制周期内需要达到的目标。 */
typedef struct
{
    bool enabled; /**< 本周期是否使能夹爪机构。 */
    bool position_mode; /**< 夹爪 M2006 是否采用位置控制模式。 */
    float m2006_vel_rad_s; /**< 夹爪的目标速度，单位：弧度每秒。 */
    float m2006_pos_rad; /**< 夹爪的目标位置，单位：弧度。 */
    bool pid_update; /**< 本次命令是否同时更新 PID 参数。 */
    upper_pid_cfg_t m2006_speed_pid; /**< 夹爪 M2006 速度环 PID 参数。 */
    upper_pid_cfg_t m2006_position_pid; /**< 夹爪 M2006 位置环 PID 参数。 */
} gripper_target_t;

/** 保存夹爪目标校验并转换后的 M2006 命令。 */
typedef struct
{
    bool enabled; /**< 校验后的夹爪命令是否允许输出。 */
    motor_cmd_t m2006; /**< 已经转换完成的机构 M2006 电机命令。 */
    bool pid_update; /**< 本次命令是否同时更新 PID 参数。 */
    upper_pid_cfg_t m2006_speed_pid; /**< 校验后的夹爪 M2006 速度环 PID 参数。 */
    upper_pid_cfg_t m2006_position_pid; /**< 校验后的夹爪 M2006 位置环 PID 参数。 */
} gripper_output_t;

/* 功能：校验夹爪目标并生成 M2006 命令；用途：把位置或速度目标转换为驱动输入；返回 true 表示转换完成。 */
bool Gripper_Calc(const gripper_target_t *target /**< 本周期夹爪机构控制目标 */,
                  gripper_output_t *output /**< 用于写出夹爪 M2006 命令的对象 */);
/* 功能：应用夹爪命令和可选 PID 参数；用途：向电机管理器提交目标与使能状态；返回 true 表示全部设置成功。 */
bool Gripper_Apply(motor_manager_t *manager /**< 需要操作的电机管理器 */,
                   const gripper_output_t *output /**< 待下发的夹爪 M2006 电机命令 */);

#endif
