/**
 * @file upper_pid.h
 * @brief 定义上层各执行机构共用的 PID 参数结构。
 */

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
