#include "pid.h"

static float clampf(float x, float lo, float hi)
{
    if (x < lo)
    {
        return lo;
    }
    if (x > hi)
    {
        return hi;
    }
    return x;
}

void pid_init(pid_t *p, float kp, float ki, float kd,
              float int_limit, float out_min, float out_max)
{
    if (p == NULL)
    {
        return;
    }
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->int_limit = int_limit;
    p->out_min = out_min;
    p->out_max = out_max;
    pid_reset(p);
}

void pid_reset(pid_t *p)
{
    if (p == NULL)
    {
        return;
    }
    p->integral = 0.0f;
    p->prev_error = 0.0f;
}

void pid_set_gains(pid_t *p, float kp, float ki, float kd)
{
    if (p == NULL)
    {
        return;
    }
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
}

float pid_step(pid_t *p, float setpoint, float measured, float dt)
{
    float error;
    float deriv;
    float out;

    if (p == NULL)
    {
        return 0.0f;
    }

    if (dt <= 0.0f)
    {
        dt = 1e-6f;   /* avoid division by zero */
    }

    error = setpoint - measured;
    p->integral += error * dt;
    if (p->int_limit > 0.0f)
    {
        p->integral = clampf(p->integral, -p->int_limit, p->int_limit);
    }
    deriv = (error - p->prev_error) / dt;
    p->prev_error = error;

    out = p->kp * error + p->ki * p->integral + p->kd * deriv;
    return clampf(out, p->out_min, p->out_max);
}
