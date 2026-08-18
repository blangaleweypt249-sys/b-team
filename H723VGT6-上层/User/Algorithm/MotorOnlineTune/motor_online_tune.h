/**
 * @file motor_online_tune.h
 * @brief 定义电机在线整定器的配置、状态和调用接口。
 */

#ifndef MOTOR_ONLINE_TUNE_H
#define MOTOR_ONLINE_TUNE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MOTOR_ONLINE_STRATEGY_EXPERT = 1,
    MOTOR_ONLINE_STRATEGY_ADAPTIVE = 2,
    MOTOR_ONLINE_STRATEGY_HYBRID = 3
} motor_online_strategy_t;

typedef enum
{
    MOTOR_ONLINE_RULE_NORMAL = 0,
    MOTOR_ONLINE_RULE_LARGE_ERROR = 1,
    MOTOR_ONLINE_RULE_SETPOINT_CROSSING = 2,
    MOTOR_ONLINE_RULE_ERROR_CHANGING_FAST = 3,
    MOTOR_ONLINE_RULE_ERROR_GROWING = 4,
    MOTOR_ONLINE_RULE_SMALL_ERROR = 5
} motor_online_rule_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
} motor_online_gains_t;

typedef struct
{
    motor_online_strategy_t strategy;
    motor_online_gains_t base_gains;
    motor_online_gains_t minimum_gains;
    motor_online_gains_t maximum_gains;
    motor_online_gains_t learning_rate;
    float adaptation_deadband;
    float normalization_epsilon;
    float adaptation_leak_per_s;
    float small_error_threshold;
    float large_error_threshold;
    float error_rate_threshold;
    float boost_factor;
    float damping_factor;
    float smoothing;
} motor_online_pid_cfg_t;

typedef struct
{
    motor_online_pid_cfg_t cfg;
    motor_online_gains_t learned_gains;
    motor_online_gains_t applied_gains;
    float previous_error;
    float adaptation_integral;
    float filtered_error_rate;
    uint8_t enabled;
    uint8_t started;
    motor_online_rule_t active_rule;
} motor_online_pid_t;

/* 功能：初始化 PID 在线调参器；用途：装载策略、边界和初始增益；返回 true 表示初始化成功。 */
bool MotorOnlinePid_Init(motor_online_pid_t *tuner,
                         const motor_online_pid_cfg_t *cfg,
                         bool enabled);
/* 功能：更新 PID 在线调参器的基准增益；用途：更换控制器标称参数并重置学习状态；返回 true 表示参数被接受。 */
bool MotorOnlinePid_SetBaseGains(motor_online_pid_t *tuner,
                                 motor_online_gains_t gains);
/* 功能：启用或关闭 PID 在线调参；用途：在固定增益与自动调整之间切换；执行后调参状态会复位。 */
void MotorOnlinePid_SetEnabled(motor_online_pid_t *tuner, bool enabled);
/* 功能：重置 PID 在线调参历史；用途：清除误差积分、变化率和活动规则；restore_gains 表示是否同时恢复基准增益。 */
void MotorOnlinePid_Reset(motor_online_pid_t *tuner, bool restore_gains);
/* 功能：执行一次 PID 在线调参更新；用途：组合自适应与专家策略生成当前增益；返回值表示本周期实际应用的增益。 */
motor_online_gains_t MotorOnlinePid_Update(motor_online_pid_t *tuner,
                                            float error,
                                            float dt_s);

typedef struct
{
    float minimum_kp;
    float maximum_kp;
    float minimum_kd;
    float maximum_kd;
    float near_error;
    float far_error;
    float velocity_scale;
    float diverging_rate;
    float stalled_rate;
    float stalled_velocity;
    float smoothing;
} motor_online_mit_cfg_t;

typedef struct
{
    motor_online_mit_cfg_t cfg;
    float base_kp;
    float base_kd;
    float applied_kp;
    float applied_kd;
    float previous_error;
    float previous_abs_error;
    float convergence_rate;
    uint8_t enabled;
    uint8_t started;
} motor_online_mit_t;

/* 功能：初始化 MIT 在线调参器；用途：装载位置刚度和阻尼的调整规则；返回 true 表示初始化成功。 */
bool MotorOnlineMit_Init(motor_online_mit_t *tuner,
                         const motor_online_mit_cfg_t *cfg,
                         bool enabled);
/* 功能：设置 MIT 控制的基准 kp、kd；用途：确定在线调整的出发点；返回 true 表示命令在允许范围内。 */
bool MotorOnlineMit_SetCommand(motor_online_mit_t *tuner,
                               float kp,
                               float kd);
/* 功能：启用或关闭 MIT 在线调参；用途：切换动态增益与原始命令增益；执行后历史状态被重置。 */
void MotorOnlineMit_SetEnabled(motor_online_mit_t *tuner, bool enabled);
/* 功能：清空 MIT 调参器的误差和收敛历史；用途：开始新的控制过程；执行后下一次更新按首帧处理。 */
void MotorOnlineMit_Reset(motor_online_mit_t *tuner);
/* 功能：根据位置误差、速度误差和收敛趋势更新 MIT 增益；用途：在线改善响应与阻尼；kp、kd 输出本周期应用值。 */
void MotorOnlineMit_Update(motor_online_mit_t *tuner,
                           float position_error,
                           float velocity_error,
                           float measured_velocity,
                           float dt_s,
                           float *kp,
                           float *kd);

#ifdef __cplusplus
}
#endif

#endif
