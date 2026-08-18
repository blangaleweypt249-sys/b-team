/**
 * @file pid.h
 * @brief 定义通用 PID 控制器的数据结构和接口。
 */

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

/* 功能：初始化 PID 控制器并保存参数；用途：建立可用的控制状态；执行后历史误差和积分均被清零。 */
void PID_Init(pid_t *pid, const pid_cfg_t *cfg);
/* 功能：清空 PID 运行状态但保留配置；用途：切换工况或停机后重新起算；执行后控制器无历史累积。 */
void PID_Reset(pid_t *pid);
/* 功能：计算一次 PID 控制输出；用途：根据目标、反馈和周期形成闭环控制量；返回值表示限幅后的控制输出。 */
float PID_Calc(pid_t *pid,
               float target,
               float feedback,
               float dt_s);

#endif
