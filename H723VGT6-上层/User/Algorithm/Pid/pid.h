/**
 * @file pid.h
 * @brief 定义通用 PID 控制器的数据结构和接口。
 */

#ifndef PID_H
#define PID_H /**< 防止 pid.h 被重复包含。 */

#include <stdbool.h>

/** 保存 PID 控制器 初始化和控制所需的配置参数。 */
typedef struct
{
    float kp; /**< 比例增益。 */
    float ki; /**< 积分增益。 */
    float kd; /**< 微分增益。 */
    float integral_limit; /**< 积分累计值的绝对值上限。 */
    float output_limit; /**< 控制器输出绝对值上限。 */
    float derivative_alpha; /**< 微分项低通滤波系数。 */
} pid_cfg_t;

/** 保存 PID 控制器 运行过程中需要集中管理的数据。 */
typedef struct
{
    pid_cfg_t cfg; /**< PID 控制器当前使用的配置参数。 */
    float integral; /**< 当前积分累计值。 */
    float prev_error; /**< 上一次控制计算的位置或速度误差。 */
    float derivative; /**< 滤波后的误差微分项。 */
    bool initialized; /**< 控制状态是否已经初始化。 */
} pid_t;

/* 功能：初始化 PID 控制器并保存参数；用途：建立可用的控制状态；执行后历史误差和积分均被清零。 */
void PID_Init(pid_t *pid /**< 需要操作的 PID 控制器 */, const pid_cfg_t *cfg /**< PID 增益及积分、输出限幅配置 */);
/* 功能：清空 PID 运行状态但保留配置；用途：切换工况或停机后重新起算；执行后控制器无历史累积。 */
void PID_Reset(pid_t *pid /**< 需要操作的 PID 控制器 */);
/* 功能：计算一次 PID 控制输出；用途：根据目标、反馈和周期形成闭环控制量；返回值表示限幅后的控制输出。 */
float PID_Calc(pid_t *pid /**< 需要操作的 PID 控制器 */,
               float target /**< PID 控制器的目标值 */,
               float feedback /**< 控制器当前使用的反馈值 */,
               float dt_s /**< 本次计算的控制周期，单位：秒 */);

#endif
