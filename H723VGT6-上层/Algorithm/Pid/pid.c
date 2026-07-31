#include "pid.h"

#include <string.h>

static float PID_Clamp(float value, float limit)
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
