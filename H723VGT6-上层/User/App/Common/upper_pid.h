/**
 * @file upper_pid.h
 * @brief 定义上层各执行机构共用的 PID 参数结构。
 */

#ifndef UPPER_PID_H
#define UPPER_PID_H /**< 防止 upper_pid.h 被重复包含。 */

/** 保存 PID 控制器 初始化和控制所需的配置参数。 */
typedef struct
{
    float kp; /**< 比例增益。 */
    float ki; /**< 积分增益。 */
    float kd; /**< 微分增益。 */
    float integral_limit; /**< 积分累计值的绝对值上限。 */
    float output_limit; /**< 控制器输出绝对值上限。 */
} upper_pid_cfg_t;

#endif
