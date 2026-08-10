#ifndef UPPER_PID_H
#define UPPER_PID_H

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
} upper_pid_cfg_t;

#endif
