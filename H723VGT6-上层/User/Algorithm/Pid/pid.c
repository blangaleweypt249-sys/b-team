/**
 * @file pid.c
 * @brief 实现通用 PID 控制器的初始化、复位和周期计算。
 */

#include "pid.h"

#include <string.h>

/* 功能：将数值限制在正负上限内；用途：抑制 PID 积分和输出饱和；返回值表示限幅后的数值。 */
static float PID_Clamp(float value /* 需要检查、限幅或编码的输入值 */, float limit /* 输入值允许达到的绝对值上限 */)
{
    if (limit <= 0.0f)
    {
        return value;
    }
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

/* 功能：初始化 PID 控制器并保存参数；用途：建立可用的控制状态；执行后历史误差和积分均被清零。 */
void PID_Init(pid_t *pid, const pid_cfg_t *cfg)
{
    if ((pid == NULL) || (cfg == NULL))
    {
        return;
    }

    (void)memset(pid, 0, sizeof(*pid));
    pid->cfg = *cfg;
    if (pid->cfg.derivative_alpha < 0.0f)
    {
        pid->cfg.derivative_alpha = 0.0f;
    }
    else if (pid->cfg.derivative_alpha > 1.0f)
    {
        pid->cfg.derivative_alpha = 1.0f;
    }
}

/* 功能：清空 PID 运行状态但保留配置；用途：切换工况或停机后重新起算；执行后控制器无历史累积。 */
void PID_Reset(pid_t *pid)
{
    pid_cfg_t cfg;

    if (pid == NULL)
    {
        return;
    }

    cfg = pid->cfg;
    (void)memset(pid, 0, sizeof(*pid));
    pid->cfg = cfg;
}

/* 功能：计算一次 PID 控制输出；用途：根据目标、反馈和周期形成闭环控制量；返回值表示限幅后的控制输出。 */
float PID_Calc(pid_t *pid,
               float target,
               float feedback,
               float dt_s)
{
    float error;
    float raw_derivative;
    float output;

    if ((pid == NULL) || (dt_s <= 0.0f))
    {
        return 0.0f;
    }

    error = target - feedback;
    pid->integral += error * dt_s;
    pid->integral = PID_Clamp(pid->integral, pid->cfg.integral_limit);

    raw_derivative = pid->initialized ?
                     (error - pid->prev_error) / dt_s : 0.0f;
    pid->derivative += pid->cfg.derivative_alpha *
                       (raw_derivative - pid->derivative);
    pid->prev_error = error;
    pid->initialized = true;

    output = pid->cfg.kp * error +
             pid->cfg.ki * pid->integral +
             pid->cfg.kd * pid->derivative;
    return PID_Clamp(output, pid->cfg.output_limit);
}
