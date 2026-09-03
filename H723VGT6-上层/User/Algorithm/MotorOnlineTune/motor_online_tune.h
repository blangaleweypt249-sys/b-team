/**
 * @file motor_online_tune.h
 * @brief 定义电机在线整定器的配置、状态和调用接口。
 */

#ifndef MOTOR_ONLINE_TUNE_H
#define MOTOR_ONLINE_TUNE_H /**< 防止 motor_online_tune.h 被重复包含。 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/** 选择 PID 在线调参采用专家规则、自适应学习或两者组合。 */
typedef enum
{
    MOTOR_ONLINE_STRATEGY_EXPERT = 1, /**< 仅使用专家规则调整控制增益。 */
    MOTOR_ONLINE_STRATEGY_ADAPTIVE = 2, /**< 仅根据误差在线学习控制增益。 */
    MOTOR_ONLINE_STRATEGY_HYBRID = 3 /**< 同时使用在线学习和专家规则。 */
} motor_online_strategy_t;

/** 标识 PID 在线调参本周期命中的误差处理规则。 */
typedef enum
{
    MOTOR_ONLINE_RULE_NORMAL = 0, /**< 误差处于正常范围，使用基准调整规则。 */
    MOTOR_ONLINE_RULE_LARGE_ERROR = 1, /**< 误差较大，需要提高响应强度。 */
    MOTOR_ONLINE_RULE_SETPOINT_CROSSING = 2, /**< 误差越过零点，需要抑制超调。 */
    MOTOR_ONLINE_RULE_ERROR_CHANGING_FAST = 3, /**< 误差变化较快，需要增加阻尼。 */
    MOTOR_ONLINE_RULE_ERROR_GROWING = 4, /**< 误差正在增大，需要增强纠偏。 */
    MOTOR_ONLINE_RULE_SMALL_ERROR = 5 /**< 误差较小，接近目标位置。 */
} motor_online_rule_t;

/** 保存 电机 运行过程中需要集中管理的数据。 */
typedef struct
{
    float kp; /**< 比例增益。 */
    float ki; /**< 积分增益。 */
    float kd; /**< 微分增益。 */
} motor_online_gains_t;

/** 保存 电机 初始化和控制所需的配置参数。 */
typedef struct
{
    motor_online_strategy_t strategy; /**< 在线调参器当前采用的增益调整策略。 */
    motor_online_gains_t base_gains; /**< 控制器开始在线调整前的基准增益。 */
    motor_online_gains_t minimum_gains; /**< 在线调整允许达到的各项最小增益。 */
    motor_online_gains_t maximum_gains; /**< 在线调整允许达到的各项最大增益。 */
    motor_online_gains_t learning_rate; /**< 各项增益沿误差梯度更新的学习率。 */
    float adaptation_deadband; /**< 停止自适应学习的误差死区。 */
    float normalization_epsilon; /**< 归一化分母的最小保护值。 */
    float adaptation_leak_per_s; /**< 学习增益每秒向基准值回落的比例。 */
    float small_error_threshold; /**< 进入小误差规则的误差上限。 */
    float large_error_threshold; /**< 进入大误差规则的误差下限。 */
    float error_rate_threshold; /**< 判定误差快速变化的变化率阈值。 */
    float boost_factor; /**< 大误差时放大比例和积分增益的系数。 */
    float damping_factor; /**< 误差快速变化时增强阻尼的系数。 */
    float smoothing; /**< 新旧增益之间进行平滑过渡的系数。 */
} motor_online_pid_cfg_t;

/** 保存 电机 运行过程中需要集中管理的数据。 */
typedef struct
{
    motor_online_pid_cfg_t cfg; /**< PID 在线调参策略、边界及平滑配置。 */
    motor_online_gains_t learned_gains; /**< 自适应算法当前学习得到的增益。 */
    motor_online_gains_t applied_gains; /**< 本控制周期实际应用的增益。 */
    float previous_error; /**< 上一次更新使用的误差。 */
    float adaptation_integral; /**< 自适应算法累计的误差积分。 */
    float filtered_error_rate; /**< 低通滤波后的误差变化率。 */
    uint8_t enabled; /**< PID 在线调参器是否启用。 */
    uint8_t started; /**< 调参器是否已经接收过首个有效样本。 */
    motor_online_rule_t active_rule; /**< 本周期实际生效的专家调整规则。 */
} motor_online_pid_t;

/* 功能：初始化 PID 在线调参器；用途：装载策略、边界和初始增益；返回 true 表示初始化成功。 */
bool MotorOnlinePid_Init(motor_online_pid_t *tuner /**< 需要初始化或更新的 PID 在线调参器 */,
                         const motor_online_pid_cfg_t *cfg /**< PID 在线调参配置 */,
                         bool enabled /**< 初始化后是否启用PID在线调参 */);
/* 功能：更新 PID 在线调参器的基准增益；用途：更换控制器标称参数并重置学习状态；返回 true 表示参数被接受。 */
bool MotorOnlinePid_SetBaseGains(motor_online_pid_t *tuner /**< 需要初始化或更新的 PID 在线调参器 */,
                                 motor_online_gains_t gains /**< 需要检查或应用的 PID 增益 */);
/* 功能：启用或关闭 PID 在线调参；用途：在固定增益与自动调整之间切换；执行后调参状态会复位。 */
void MotorOnlinePid_SetEnabled(motor_online_pid_t *tuner /**< 需要初始化或更新的 PID 在线调参器 */, bool enabled /**< 是否启用PID在线调参 */);
/* 功能：重置 PID 在线调参历史；用途：清除误差积分、变化率和活动规则；restore_gains 表示是否同时恢复基准增益。 */
void MotorOnlinePid_Reset(motor_online_pid_t *tuner /**< 需要初始化或更新的 PID 在线调参器 */, bool restore_gains /**< 是否同时恢复到基准增益 */);
/* 功能：执行一次 PID 在线调参更新；用途：组合自适应与专家策略生成当前增益；返回值表示本周期实际应用的增益。 */
motor_online_gains_t MotorOnlinePid_Update(motor_online_pid_t *tuner /**< 需要初始化或更新的 PID 在线调参器 */,
                                            float error /**< 当前控制误差 */,
                                            float dt_s /**< 本次计算的控制周期，单位：秒 */);

/** 保存 电机 初始化和控制所需的配置参数。 */
typedef struct
{
    float minimum_kp; /**< MIT 在线调整允许使用的最小比例增益。 */
    float maximum_kp; /**< MIT 在线调整允许使用的最大比例增益。 */
    float minimum_kd; /**< MIT 在线调整允许使用的最小微分增益。 */
    float maximum_kd; /**< MIT 在线调整允许使用的最大微分增益。 */
    float near_error; /**< 判定 J4310 接近目标位置的误差阈值，单位：弧度。 */
    float far_error; /**< 判定 J4310 远离目标位置的误差阈值，单位：弧度。 */
    float velocity_scale; /**< 将实测速度归一化后参与 MIT 增益调整的比例。 */
    float diverging_rate; /**< 判定位置误差正在发散的变化率阈值。 */
    float stalled_rate; /**< 判定位置误差基本不再收敛的变化率阈值。 */
    float stalled_velocity; /**< 判定 J4310 接近静止的速度阈值，单位：弧度每秒。 */
    float smoothing; /**< 新旧增益之间进行平滑过渡的系数。 */
} motor_online_mit_cfg_t;

/** 保存 电机 运行过程中需要集中管理的数据。 */
typedef struct
{
    motor_online_mit_cfg_t cfg; /**< MIT 在线调参阈值、增益及平滑配置。 */
    float base_kp; /**< 在线调整前的基准比例增益。 */
    float base_kd; /**< 在线调整前的基准微分增益。 */
    float applied_kp; /**< 本控制周期实际应用的比例增益。 */
    float applied_kd; /**< 本控制周期实际应用的微分增益。 */
    float previous_error; /**< 上一次更新使用的误差。 */
    float previous_abs_error; /**< 上一次更新时误差的绝对值。 */
    float convergence_rate; /**< 位置误差绝对值的收敛变化率。 */
    uint8_t enabled; /**< MIT 在线调参器是否启用。 */
    uint8_t started; /**< 调参器是否已经接收过首个有效样本。 */
} motor_online_mit_t;

/* 功能：初始化 MIT 在线调参器；用途：装载位置刚度和阻尼的调整规则；返回 true 表示初始化成功。 */
bool MotorOnlineMit_Init(motor_online_mit_t *tuner /**< 需要初始化或更新的 MIT 在线调参器 */,
                         const motor_online_mit_cfg_t *cfg /**< MIT 在线调参配置 */,
                         bool enabled /**< 初始化后是否启用MIT在线调参 */);
/* 功能：设置 MIT 控制的基准 kp、kd；用途：确定在线调整的出发点；返回 true 表示命令在允许范围内。 */
bool MotorOnlineMit_SetCommand(motor_online_mit_t *tuner /**< 需要初始化或更新的 MIT 在线调参器 */,
                               float kp /**< 比例增益 */,
                               float kd /**< 微分增益 */);
/* 功能：启用或关闭 MIT 在线调参；用途：切换动态增益与原始命令增益；执行后历史状态被重置。 */
void MotorOnlineMit_SetEnabled(motor_online_mit_t *tuner /**< 需要初始化或更新的 MIT 在线调参器 */, bool enabled /**< 是否启用MIT在线调参 */);
/* 功能：清空 MIT 调参器的误差和收敛历史；用途：开始新的控制过程；执行后下一次更新按首帧处理。 */
void MotorOnlineMit_Reset(motor_online_mit_t *tuner /**< 需要初始化或更新的 MIT 在线调参器 */);
/* 功能：根据位置误差、速度误差和收敛趋势更新 MIT 增益；用途：在线改善响应与阻尼；kp、kd 输出本周期应用值。 */
void MotorOnlineMit_Update(motor_online_mit_t *tuner /**< 需要初始化或更新的 MIT 在线调参器 */,
                           float position_error /**< 当前目标位置与反馈位置的误差 */,
                           float velocity_error /**< 当前目标速度与反馈速度的误差 */,
                           float measured_velocity /**< 电机当前实测速度 */,
                           float dt_s /**< 本次计算的控制周期，单位：秒 */,
                           float *kp /**< 比例增益 */,
                           float *kd /**< 微分增益 */);

#ifdef __cplusplus
}
#endif

#endif
