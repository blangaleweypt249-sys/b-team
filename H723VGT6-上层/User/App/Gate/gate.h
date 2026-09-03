/**
 * @file gate.h
 * @brief 定义闸门目标、输出数据结构和控制接口。
 */

#ifndef GATE_H
#define GATE_H /**< 防止 gate.h 被重复包含。 */

#include <stdbool.h>

#include "motor_manager.h"
#include "upper_pid.h"

#define UPPER_GATE_M2006_POSITION_LIMIT_RAD       6.28318530718f /**< 挡板机构位置目标绝对值的上限，单位：弧度。 */
#define UPPER_GATE_M2006_POSITION_EPSILON_RAD     0.000001f /**< 比较挡板机构位置目标是否变化时使用的容差，单位：弧度。 */
#define UPPER_GATE_M2006_POSITION_VEL_LIMIT_RAD_S 5.235987756f /**< 挡板机构执行位置轨迹时允许的最大速度，单位：弧度每秒。 */

/** 描述挡板机构在当前控制周期内需要达到的目标。 */
typedef struct
{
    bool enabled; /**< 本周期是否使能挡板机构。 */
    bool position_mode; /**< 挡板 M2006 是否采用位置控制模式。 */
    float m2006_vel_rad_s; /**< 挡板的目标速度，单位：弧度每秒。 */
    float m2006_pos_rad; /**< 挡板的目标位置，单位：弧度。 */
    bool pid_update; /**< 本次命令是否同时更新 PID 参数。 */
    upper_pid_cfg_t m2006_speed_pid; /**< 挡板 M2006 速度环 PID 参数。 */
    upper_pid_cfg_t m2006_position_pid; /**< 挡板 M2006 位置环 PID 参数。 */
} gate_target_t;

/** 保存挡板目标校验并转换后的 M2006 命令。 */
typedef struct
{
    bool enabled; /**< 校验后的挡板命令是否允许输出。 */
    motor_cmd_t m2006; /**< 已经转换完成的机构 M2006 电机命令。 */
    bool pid_update; /**< 本次命令是否同时更新 PID 参数。 */
    upper_pid_cfg_t m2006_speed_pid; /**< 校验后的挡板 M2006 速度环 PID 参数。 */
    upper_pid_cfg_t m2006_position_pid; /**< 校验后的挡板 M2006 位置环 PID 参数。 */
} gate_output_t;

/* 功能：校验闸门目标并生成 M2006 命令；用途：把位置或速度需求转换为驱动输入。 */
bool Gate_Calc(const gate_target_t *target /**< 本周期挡板机构控制目标 */, gate_output_t *output /**< 用于写出挡板 M2006 命令的对象 */);
/* 功能：应用闸门命令和可选 PID 参数；用途：提交目标并设置电机使能状态。 */
bool Gate_Apply(motor_manager_t *manager /**< 需要操作的电机管理器 */, const gate_output_t *output /**< 待下发的挡板 M2006 电机命令 */);

#endif
