#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
    float derivative_alpha;
} pid_cfg_t;

typedef struct
{
    pid_cfg_t cfg;
    float integral;
    float prev_error;
    float derivative;
    bool initialized;
} pid_t;

void PID_Init(pid_t *pid, const pid_cfg_t *cfg);
void PID_Reset(pid_t *pid);
float PID_Calc(pid_t *pid,
               float target,
               float feedback,
               float dt_s);

#endif
