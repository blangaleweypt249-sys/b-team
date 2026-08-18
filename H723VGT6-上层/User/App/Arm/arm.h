/**
 * @file arm.h
 * @brief 定义机械臂目标、输出数据结构和控制接口。
 */

#ifndef ARM_H
#define ARM_H

#include <stdbool.h>

#include "upper_motor_port.h"
#include "upper_pid.h"

#define UPPER_ARM_M3508_COUNT                  2U

/* 机械臂命令边界；MIT 映射限值必须与 J4310 持久配置一致。 */
#define UPPER_J4310_POSITION_MAX_RAD           12.5f
#define UPPER_J4310_VELOCITY_MAX_RAD_S         30.0f
#define UPPER_J4310_TORQUE_MAP_MAX_NM          10.0f
#define UPPER_J4310_KP_MAX                     49.0f
#define UPPER_J4310_KD_MAX                      0.95f

#define UPPER_M3508_POSITION_MIN_RAD            0.0f
#define UPPER_M3508_POSITION_MAX_RAD            20.9439510239f
#define UPPER_M3508_POSITION_VEL_LIMIT_RAD_S    15.708f

typedef struct
{
    bool enabled;
    bool j4310_commanded;
    bool m3508_enabled;
    bool position_mode;
    float grip_pos_rad;
    float grip_vel_rad_s;
    float grip_kp;
    float grip_kd;
    float grip_torque_nm;
    float grip_torque_limit_nm;
    float m3508_vel_rad_s[UPPER_ARM_M3508_COUNT];
    float m3508_pos_rad[UPPER_ARM_M3508_COUNT];
    bool pid_update;
    upper_pid_cfg_t m3508_speed_pid;
    upper_pid_cfg_t m3508_position_pid;
} arm_target_t;

typedef struct
{
    bool enabled;
    bool m3508_enabled;
    motor_cmd_t j4310;
    motor_cmd_t m3508[UPPER_ARM_M3508_COUNT];
    bool pid_update;
    float j4310_torque_limit_nm;
    upper_pid_cfg_t m3508_speed_pid;
    upper_pid_cfg_t m3508_position_pid;
} arm_output_t;

/* 功能：校验机械臂目标并转换为各电机命令；用途：生成单周期可统一下发的输出快照；返回 true 表示目标合法且转换完成。 */
bool Arm_Calc(const arm_target_t *target, arm_output_t *output);
/* 功能：应用机械臂输出及可选 PID 更新；用途：把 J4310 和 M3508 命令提交给电机管理器；返回 true 表示全部设置成功。 */
bool Arm_Apply(motor_manager_t *manager, const arm_output_t *output);

#endif
