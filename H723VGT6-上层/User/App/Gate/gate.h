/**
 * @file gate.h
 * @brief 定义闸门目标、输出数据结构和控制接口。
 */

#ifndef GATE_H
#define GATE_H

#include <stdbool.h>

#include "motor_manager.h"
#include "upper_pid.h"

#define UPPER_GATE_M2006_POSITION_LIMIT_RAD       6.28318530718f
#define UPPER_GATE_M2006_POSITION_EPSILON_RAD     0.000001f
#define UPPER_GATE_M2006_POSITION_VEL_LIMIT_RAD_S 5.235987756f

typedef struct
{
    bool enabled;
    bool position_mode;
    float m2006_vel_rad_s;
    float m2006_pos_rad;
    bool pid_update;
    upper_pid_cfg_t m2006_speed_pid;
    upper_pid_cfg_t m2006_position_pid;
} gate_target_t;

typedef struct
{
    bool enabled;
    motor_cmd_t m2006;
    bool pid_update;
    upper_pid_cfg_t m2006_speed_pid;
    upper_pid_cfg_t m2006_position_pid;
} gate_output_t;

/* 功能：校验闸门目标并生成 M2006 命令；用途：把位置或速度需求转换为驱动输入。 */
bool Gate_Calc(const gate_target_t *target, gate_output_t *output);
/* 功能：应用闸门命令和可选 PID 参数；用途：提交目标并设置电机使能状态。 */
bool Gate_Apply(motor_manager_t *manager, const gate_output_t *output);

#endif
