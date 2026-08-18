/**
 * @file gripper.h
 * @brief 定义夹爪目标、输出数据结构和控制接口。
 */

#ifndef GRIPPER_H
#define GRIPPER_H

#include <stdbool.h>

#include "motor_manager.h"
#include "upper_pid.h"

/* 夹爪在 C610 输出后还有 2:1 的额外减速。 */
#define UPPER_GRIPPER_M2006_POSITION_LIMIT_RAD  12.56637061436f
#define UPPER_GRIPPER_M2006_POSITION_EPSILON_RAD 0.000001f
#define UPPER_GRIPPER_M2006_POSITION_VEL_LIMIT_RAD_S 5.235987756f

typedef struct
{
    bool enabled;
    bool position_mode;
    float m2006_vel_rad_s;
    float m2006_pos_rad;
    bool pid_update;
    upper_pid_cfg_t m2006_speed_pid;
    upper_pid_cfg_t m2006_position_pid;
} gripper_target_t;

typedef struct
{
    bool enabled;
    motor_cmd_t m2006;
    bool pid_update;
    upper_pid_cfg_t m2006_speed_pid;
    upper_pid_cfg_t m2006_position_pid;
} gripper_output_t;

/* 功能：校验夹爪目标并生成 M2006 命令；用途：把位置或速度目标转换为驱动输入；返回 true 表示转换完成。 */
bool Gripper_Calc(const gripper_target_t *target,
                  gripper_output_t *output);
/* 功能：应用夹爪命令和可选 PID 参数；用途：向电机管理器提交目标与使能状态；返回 true 表示全部设置成功。 */
bool Gripper_Apply(motor_manager_t *manager,
                   const gripper_output_t *output);

#endif
