/**
 * @file gate.c
 * @brief 实现闸门目标到 M2006 命令的计算与下发。
 */

#include "gate.h"

#include <math.h>
#include <string.h>

#include "bsp_can.h"
#include "m2006.h"
#include "upper_motor_port.h"

/* 功能：校验闸门电机 PID 参数是否合法；参数 cfg 为待检查配置；返回 true 表示参数完整且位于有效范围。 */
static bool Gate_PidValid(const upper_pid_cfg_t *cfg /* 初始化或更新时使用的配置参数 */)
{
    return (cfg != NULL) && isfinite(cfg->kp) && isfinite(cfg->ki) &&
           isfinite(cfg->kd) && isfinite(cfg->integral_limit) &&
           isfinite(cfg->output_limit) && (cfg->kp >= 0.0f) &&
           (cfg->ki >= 0.0f) && (cfg->kd >= 0.0f) &&
           (cfg->integral_limit >= 0.0f) && (cfg->output_limit > 0.0f);
}

/* 功能：比较两组闸门电机 PID 配置是否一致；参数 left、right 为待比较配置；返回 true 表示所有字段相同。 */
static bool Gate_PidEqual(const upper_pid_cfg_t *left /* 函数读取或写入的对象地址 */,
                          const upper_pid_cfg_t *right /* 函数读取或写入的对象地址 */)
{
    return memcmp(left, right, sizeof(*left)) == 0;
}

/* 功能：将数值限制在指定区间内；参数 value 为输入值，lower、upper 为上下限；返回限幅后的结果。 */
static float Gate_Clamp(float value /* 需要检查、限幅或编码的输入值 */, float lower /* 允许输出的下限 */, float upper /* 允许输出的上限 */)
{
    if (value < lower)
    {
        return lower;
    }
    if (value > upper)
    {
        return upper;
    }
    return value;
}

/* 功能：校验闸门控制目标并生成 M2006 电机命令；参数 target 为目标，output 为输出；返回 true 表示计算成功。 */
bool Gate_Calc(const gate_target_t *target, gate_output_t *output)
{
    if ((target == NULL) || (output == NULL))
    {
        return false;
    }

    (void)memset(output, 0, sizeof(*output));
    if (!target->enabled)
    {
        output->m2006.mode = MOTOR_CMD_STOP;
        return true;
    }

    if (target->position_mode)
    {
        if (!isfinite(target->m2006_pos_rad) ||
            (target->m2006_pos_rad <
             -UPPER_GATE_M2006_POSITION_LIMIT_RAD -
             UPPER_GATE_M2006_POSITION_EPSILON_RAD) ||
            (target->m2006_pos_rad >
             UPPER_GATE_M2006_POSITION_LIMIT_RAD +
             UPPER_GATE_M2006_POSITION_EPSILON_RAD))
        {
            return false;
        }
    }
    else if (!isfinite(target->m2006_vel_rad_s) ||
             (target->m2006_vel_rad_s <
              -UPPER_GATE_M2006_POSITION_VEL_LIMIT_RAD_S) ||
             (target->m2006_vel_rad_s >
              UPPER_GATE_M2006_POSITION_VEL_LIMIT_RAD_S))
    {
        return false;
    }
    if (target->pid_update &&
        (!Gate_PidValid(&target->m2006_speed_pid) ||
         !Gate_PidValid(&target->m2006_position_pid)))
    {
        return false;
    }

    output->enabled = target->enabled;
    output->pid_update = target->pid_update;
    output->m2006_speed_pid = target->m2006_speed_pid;
    output->m2006_position_pid = target->m2006_position_pid;
    output->m2006 = (motor_cmd_t)
    {
        .mode = target->position_mode ? MOTOR_CMD_POSITION : MOTOR_CMD_VELOCITY,
        .pos_rad = target->position_mode ?
                   Gate_Clamp(target->m2006_pos_rad,
                              -UPPER_GATE_M2006_POSITION_LIMIT_RAD,
                              UPPER_GATE_M2006_POSITION_LIMIT_RAD) : 0.0f,
        .vel_rad_s = target->m2006_vel_rad_s
    };
    return true;
}

/* 功能：把闸门控制输出和更新后的 PID 参数写入电机管理器；参数 manager、output 分别为管理器和输出；返回 true 表示下发成功。 */
bool Gate_Apply(motor_manager_t *manager, const gate_output_t *output)
{
    bool success;
    static bool pid_applied;
    static upper_pid_cfg_t applied_speed_pid;
    static upper_pid_cfg_t applied_position_pid;

    if ((manager == NULL) || (output == NULL))
    {
        return false;
    }
    if (output->pid_update &&
        (!pid_applied ||
         !Gate_PidEqual(&applied_speed_pid, &output->m2006_speed_pid) ||
         !Gate_PidEqual(&applied_position_pid, &output->m2006_position_pid)))
    {
        m2006_pid_cfg_t speed_pid;
        m2006_pid_cfg_t position_pid;

        speed_pid = (m2006_pid_cfg_t)
        {
            output->m2006_speed_pid.kp,
            output->m2006_speed_pid.ki,
            output->m2006_speed_pid.kd,
            output->m2006_speed_pid.integral_limit,
            output->m2006_speed_pid.output_limit
        };
        position_pid = (m2006_pid_cfg_t)
        {
            output->m2006_position_pid.kp,
            output->m2006_position_pid.ki,
            output->m2006_position_pid.kd,
            output->m2006_position_pid.integral_limit,
            output->m2006_position_pid.output_limit
        };
        success = M2006_SetSpeedPid(CAN_BUS_AUX,
                                    NODE_GATE_M2006,
                                    &speed_pid);
        success = M2006_SetPositionPid(CAN_BUS_AUX,
                                       NODE_GATE_M2006,
                                       &position_pid) && success;
        if (!success)
        {
            return false;
        }
        applied_speed_pid = output->m2006_speed_pid;
        applied_position_pid = output->m2006_position_pid;
        pid_applied = true;
    }
    success = MotorManager_SetCmd(manager,
                                  UPPER_MOTOR_GATE_M2006,
                                  &output->m2006);
    success = MotorManager_SetEnabled(manager,
                                      UPPER_MOTOR_GATE_M2006,
                                      output->enabled) && success;
    return success;
}
