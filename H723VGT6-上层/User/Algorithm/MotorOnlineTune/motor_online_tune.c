/**
 * @file motor_online_tune.c
 * @brief 实现电机 PID 与 MIT 控制参数的在线整定算法。
 */

#include "motor_online_tune.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

/* 功能：判断浮点数是否有限；用途：拦截 NaN 和无穷值；返回 true 表示该值可参与调参计算。 */
static bool MotorOnline_IsFinite(float value /* 需要检查、限幅或编码的输入值 */)
{
    return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX);
}

/* 功能：把数值限制在给定区间；用途：约束在线调参的增益和比例；返回值表示限幅结果。 */
static float MotorOnline_Clamp(float value /* 需要检查、限幅或编码的输入值 */, float min /* 允许输出的下限 */, float max /* 允许输出的上限 */)
{
    if (value < min)
    {
        return min;
    }
    if (value > max)
    {
        return max;
    }
    return value;
}

/* 功能：按比例将当前值平滑逼近目标值；用途：避免在线增益突变；返回值表示本周期的新值。 */
static float MotorOnline_Approach(float current /* 当前需要平滑或换算的数值 */, float target /* 本次需要应用的控制目标 */, float alpha /* 新旧数值之间的平滑系数 */)
{
    return current + (target - current) * alpha;
}

/* 功能：检查一组 PID 增益是否均为有限数；用途：验证调参输入；返回 true 表示 kp、ki、kd 都有效。 */
static bool MotorOnline_GainsFinite(motor_online_gains_t gains /* 需要检查或应用的 PID 增益 */)
{
    return MotorOnline_IsFinite(gains.kp) &&
           MotorOnline_IsFinite(gains.ki) &&
           MotorOnline_IsFinite(gains.kd);
}

/* 功能：按配置上下限约束 PID 增益；用途：保证在线调整不越过安全范围；返回值表示约束后的增益组。 */
static motor_online_gains_t MotorOnline_ClampGains(
    motor_online_gains_t gains /* 需要检查或应用的 PID 增益 */,
    const motor_online_pid_cfg_t *cfg /* 初始化或更新时使用的配置参数 */)
{
    gains.kp = MotorOnline_Clamp(gains.kp,
                                 cfg->minimum_gains.kp,
                                 cfg->maximum_gains.kp);
    gains.ki = MotorOnline_Clamp(gains.ki,
                                 cfg->minimum_gains.ki,
                                 cfg->maximum_gains.ki);
    gains.kd = MotorOnline_Clamp(gains.kd,
                                 cfg->minimum_gains.kd,
                                 cfg->maximum_gains.kd);
    return gains;
}

/* 功能：完整校验 PID 在线调参配置；用途：在初始化前拒绝矛盾或非法参数；返回 true 表示配置可用。 */
static bool MotorOnline_PidCfgValid(const motor_online_pid_cfg_t *cfg /* 初始化或更新时使用的配置参数 */)
{
    if ((cfg == NULL) ||
        ((cfg->strategy != MOTOR_ONLINE_STRATEGY_EXPERT) &&
         (cfg->strategy != MOTOR_ONLINE_STRATEGY_ADAPTIVE) &&
         (cfg->strategy != MOTOR_ONLINE_STRATEGY_HYBRID)) ||
        !MotorOnline_GainsFinite(cfg->base_gains) ||
        !MotorOnline_GainsFinite(cfg->minimum_gains) ||
        !MotorOnline_GainsFinite(cfg->maximum_gains) ||
        !MotorOnline_GainsFinite(cfg->learning_rate) ||
        (cfg->minimum_gains.kp < 0.0f) ||
        (cfg->minimum_gains.ki < 0.0f) ||
        (cfg->minimum_gains.kd < 0.0f) ||
        (cfg->maximum_gains.kp < cfg->minimum_gains.kp) ||
        (cfg->maximum_gains.ki < cfg->minimum_gains.ki) ||
        (cfg->maximum_gains.kd < cfg->minimum_gains.kd) ||
        (cfg->base_gains.kp < cfg->minimum_gains.kp) ||
        (cfg->base_gains.kp > cfg->maximum_gains.kp) ||
        (cfg->base_gains.ki < cfg->minimum_gains.ki) ||
        (cfg->base_gains.ki > cfg->maximum_gains.ki) ||
        (cfg->base_gains.kd < cfg->minimum_gains.kd) ||
        (cfg->base_gains.kd > cfg->maximum_gains.kd) ||
        (cfg->learning_rate.kp < 0.0f) ||
        (cfg->learning_rate.ki < 0.0f) ||
        (cfg->learning_rate.kd < 0.0f) ||
        !MotorOnline_IsFinite(cfg->adaptation_deadband) ||
        (cfg->adaptation_deadband < 0.0f) ||
        !MotorOnline_IsFinite(cfg->normalization_epsilon) ||
        (cfg->normalization_epsilon <= 0.0f) ||
        !MotorOnline_IsFinite(cfg->adaptation_leak_per_s) ||
        (cfg->adaptation_leak_per_s < 0.0f) ||
        !MotorOnline_IsFinite(cfg->small_error_threshold) ||
        (cfg->small_error_threshold < 0.0f) ||
        !MotorOnline_IsFinite(cfg->large_error_threshold) ||
        (cfg->large_error_threshold <= cfg->small_error_threshold) ||
        !MotorOnline_IsFinite(cfg->error_rate_threshold) ||
        (cfg->error_rate_threshold < 0.0f) ||
        !MotorOnline_IsFinite(cfg->boost_factor) ||
        (cfg->boost_factor < 1.0f) ||
        !MotorOnline_IsFinite(cfg->damping_factor) ||
        (cfg->damping_factor <= 0.0f) ||
        (cfg->damping_factor > 1.0f) ||
        !MotorOnline_IsFinite(cfg->smoothing) ||
        (cfg->smoothing <= 0.0f) ||
        (cfg->smoothing > 1.0f))
    {
        return false;
    }
    return true;
}

/* 功能：初始化 PID 在线调参器；用途：装载策略、边界和初始增益；返回 true 表示初始化成功。 */
bool MotorOnlinePid_Init(motor_online_pid_t *tuner,
                         const motor_online_pid_cfg_t *cfg,
                         bool enabled)
{
    if ((tuner == NULL) || !MotorOnline_PidCfgValid(cfg))
    {
        return false;
    }
    tuner->cfg = *cfg;
    tuner->learned_gains = cfg->base_gains;
    tuner->applied_gains = cfg->base_gains;
    tuner->enabled = enabled ? 1U : 0U;
    MotorOnlinePid_Reset(tuner, false);
    return true;
}

/* 功能：更新 PID 在线调参器的基准增益；用途：更换控制器标称参数并重置学习状态；返回 true 表示参数被接受。 */
bool MotorOnlinePid_SetBaseGains(motor_online_pid_t *tuner,
                                 motor_online_gains_t gains)
{
    if ((tuner == NULL) || !MotorOnline_GainsFinite(gains) ||
        (gains.kp < tuner->cfg.minimum_gains.kp) ||
        (gains.kp > tuner->cfg.maximum_gains.kp) ||
        (gains.ki < tuner->cfg.minimum_gains.ki) ||
        (gains.ki > tuner->cfg.maximum_gains.ki) ||
        (gains.kd < tuner->cfg.minimum_gains.kd) ||
        (gains.kd > tuner->cfg.maximum_gains.kd))
    {
        return false;
    }
    tuner->cfg.base_gains = gains;
    MotorOnlinePid_Reset(tuner, true);
    return true;
}

/* 功能：启用或关闭 PID 在线调参；用途：在固定增益与自动调整之间切换；执行后调参状态会复位。 */
void MotorOnlinePid_SetEnabled(motor_online_pid_t *tuner, bool enabled)
{
    if (tuner == NULL)
    {
        return;
    }
    tuner->enabled = enabled ? 1U : 0U;
    MotorOnlinePid_Reset(tuner, true);
}

/* 功能：重置 PID 在线调参历史；用途：清除误差积分、变化率和活动规则；restore_gains 表示是否同时恢复基准增益。 */
void MotorOnlinePid_Reset(motor_online_pid_t *tuner, bool restore_gains)
{
    if (tuner == NULL)
    {
        return;
    }
    if (restore_gains)
    {
        tuner->learned_gains = tuner->cfg.base_gains;
        tuner->applied_gains = tuner->cfg.base_gains;
    }
    tuner->previous_error = 0.0f;
    tuner->adaptation_integral = 0.0f;
    tuner->filtered_error_rate = 0.0f;
    tuner->started = 0U;
    tuner->active_rule = MOTOR_ONLINE_RULE_NORMAL;
}

/* 功能：依据误差梯度自适应修正 PID 增益；用途：实现在线学习和增益回泄；结果写入调参器的 learned_gains。 */
static void MotorOnline_Adapt(motor_online_pid_t *tuner /* 需要操作的在线调参器 */,
                              float error /* 当前控制误差 */,
                              float error_rate /* 当前误差变化率 */,
                              float dt_s /* 本次计算的控制周期，单位：秒 */)
{
    motor_online_gains_t gains;
    float integral_limit;
    float proportional_basis;
    float integral_basis;
    float derivative_scale;
    float derivative_basis;
    float normalization;
    float common_gradient;
    float leak;

    gains = tuner->learned_gains;
    integral_limit = tuner->cfg.large_error_threshold * 5.0f;
    tuner->adaptation_integral = MotorOnline_Clamp(
        tuner->adaptation_integral + error * dt_s,
        -integral_limit,
        integral_limit);
    if (fabsf(error) > tuner->cfg.adaptation_deadband)
    {
        proportional_basis = error / tuner->cfg.large_error_threshold;
        integral_basis = tuner->adaptation_integral /
                         tuner->cfg.large_error_threshold;
        derivative_scale = (tuner->cfg.error_rate_threshold > 0.0f) ?
                           tuner->cfg.error_rate_threshold :
                           tuner->cfg.large_error_threshold;
        derivative_basis = error_rate / derivative_scale;
        normalization = tuner->cfg.normalization_epsilon +
                        proportional_basis * proportional_basis +
                        integral_basis * integral_basis +
                        derivative_basis * derivative_basis;
        common_gradient = proportional_basis / normalization;
        gains.kp += tuner->cfg.learning_rate.kp * common_gradient *
                    proportional_basis;
        gains.ki += tuner->cfg.learning_rate.ki * common_gradient *
                    integral_basis;
        gains.kd += tuner->cfg.learning_rate.kd * common_gradient *
                    derivative_basis;
    }
    leak = tuner->cfg.adaptation_leak_per_s * dt_s;
    gains.kp += (tuner->cfg.base_gains.kp - gains.kp) * leak;
    gains.ki += (tuner->cfg.base_gains.ki - gains.ki) * leak;
    gains.kd += (tuner->cfg.base_gains.kd - gains.kd) * leak;
    tuner->learned_gains = MotorOnline_ClampGains(gains, &tuner->cfg);
}

/* 功能：按误差工况应用专家规则；用途：针对大误差、过零和快速变化调整增益；返回值表示规则处理后的目标增益。 */
static motor_online_gains_t MotorOnline_ApplyExpert(
    motor_online_pid_t *tuner /* 需要操作的在线调参器 */,
    motor_online_gains_t source /* 规则调整前的原始 PID 增益 */,
    float error /* 当前控制误差 */,
    float error_rate /* 当前误差变化率 */)
{
    motor_online_gains_t target;
    float absolute_error;
    bool crossed;

    target = source;
    absolute_error = fabsf(error);
    crossed = (tuner->started != 0U) &&
              ((error * tuner->previous_error) < 0.0f);
    tuner->active_rule = MOTOR_ONLINE_RULE_NORMAL;
    if (absolute_error >= tuner->cfg.large_error_threshold)
    {
        target.kp *= tuner->cfg.boost_factor;
        target.ki *= tuner->cfg.damping_factor;
        target.kd *= 0.8f;
        tuner->active_rule = MOTOR_ONLINE_RULE_LARGE_ERROR;
    }
    else if (crossed)
    {
        target.kp *= tuner->cfg.damping_factor;
        target.ki *= tuner->cfg.damping_factor;
        target.kd *= 2.0f - tuner->cfg.damping_factor;
        tuner->active_rule = MOTOR_ONLINE_RULE_SETPOINT_CROSSING;
    }
    else if ((tuner->cfg.error_rate_threshold > 0.0f) &&
             (fabsf(error_rate) >= tuner->cfg.error_rate_threshold))
    {
        target.kp *= tuner->cfg.damping_factor;
        target.ki *= tuner->cfg.damping_factor;
        target.kd *= tuner->cfg.boost_factor;
        tuner->active_rule = MOTOR_ONLINE_RULE_ERROR_CHANGING_FAST;
    }
    else if ((absolute_error > tuner->cfg.small_error_threshold) &&
             ((error * error_rate) > 0.0f))
    {
        target.kp *= tuner->cfg.boost_factor;
        target.kd *= 1.1f;
        tuner->active_rule = MOTOR_ONLINE_RULE_ERROR_GROWING;
    }
    else if (absolute_error <= tuner->cfg.small_error_threshold)
    {
        target.ki *= 1.1f;
        target.kd *= 1.1f;
        tuner->active_rule = MOTOR_ONLINE_RULE_SMALL_ERROR;
    }
    return MotorOnline_ClampGains(target, &tuner->cfg);
}

/* 功能：执行一次 PID 在线调参更新；用途：组合自适应与专家策略生成当前增益；返回值表示本周期实际应用的增益。 */
motor_online_gains_t MotorOnlinePid_Update(motor_online_pid_t *tuner,
                                            float error,
                                            float dt_s)
{
    motor_online_gains_t target;
    float error_rate;

    if ((tuner == NULL) || !MotorOnline_IsFinite(error) ||
        !MotorOnline_IsFinite(dt_s) || (dt_s <= 0.0f))
    {
        const motor_online_gains_t zero = {0.0f, 0.0f, 0.0f};
        return zero;
    }
    if (tuner->enabled == 0U)
    {
        tuner->applied_gains = tuner->cfg.base_gains;
        return tuner->applied_gains;
    }

    error_rate = 0.0f;
    if (tuner->started != 0U)
    {
        error_rate = (error - tuner->previous_error) / dt_s;
        tuner->filtered_error_rate =
            0.8f * tuner->filtered_error_rate + 0.2f * error_rate;
    }
    error_rate = tuner->filtered_error_rate;
    if ((tuner->cfg.strategy == MOTOR_ONLINE_STRATEGY_ADAPTIVE) ||
        (tuner->cfg.strategy == MOTOR_ONLINE_STRATEGY_HYBRID))
    {
        MotorOnline_Adapt(tuner, error, error_rate, dt_s);
    }
    else
    {
        tuner->learned_gains = tuner->cfg.base_gains;
    }

    target = tuner->learned_gains;
    if ((tuner->cfg.strategy == MOTOR_ONLINE_STRATEGY_EXPERT) ||
        (tuner->cfg.strategy == MOTOR_ONLINE_STRATEGY_HYBRID))
    {
        target = MotorOnline_ApplyExpert(tuner, target, error, error_rate);
    }
    else
    {
        tuner->active_rule = MOTOR_ONLINE_RULE_NORMAL;
    }
    tuner->applied_gains.kp = MotorOnline_Approach(
        tuner->applied_gains.kp, target.kp, tuner->cfg.smoothing);
    tuner->applied_gains.ki = MotorOnline_Approach(
        tuner->applied_gains.ki, target.ki, tuner->cfg.smoothing);
    tuner->applied_gains.kd = MotorOnline_Approach(
        tuner->applied_gains.kd, target.kd, tuner->cfg.smoothing);
    tuner->applied_gains = MotorOnline_ClampGains(tuner->applied_gains,
                                                  &tuner->cfg);
    tuner->previous_error = error;
    tuner->started = 1U;
    return tuner->applied_gains;
}

/* 功能：校验 MIT 在线调参配置；用途：检查增益边界、误差阈值和滤波参数；返回 true 表示配置可用。 */
static bool MotorOnline_MitCfgValid(const motor_online_mit_cfg_t *cfg /* 初始化或更新时使用的配置参数 */)
{
    return (cfg != NULL) && MotorOnline_IsFinite(cfg->minimum_kp) &&
           MotorOnline_IsFinite(cfg->maximum_kp) &&
           MotorOnline_IsFinite(cfg->minimum_kd) &&
           MotorOnline_IsFinite(cfg->maximum_kd) &&
           MotorOnline_IsFinite(cfg->near_error) &&
           MotorOnline_IsFinite(cfg->far_error) &&
           MotorOnline_IsFinite(cfg->velocity_scale) &&
           MotorOnline_IsFinite(cfg->diverging_rate) &&
           MotorOnline_IsFinite(cfg->stalled_rate) &&
           MotorOnline_IsFinite(cfg->stalled_velocity) &&
           MotorOnline_IsFinite(cfg->smoothing) &&
           (cfg->minimum_kp >= 0.0f) &&
           (cfg->maximum_kp >= cfg->minimum_kp) &&
           (cfg->minimum_kd >= 0.0f) &&
           (cfg->maximum_kd >= cfg->minimum_kd) &&
           (cfg->near_error >= 0.0f) &&
           (cfg->far_error > cfg->near_error) &&
           (cfg->velocity_scale > 0.0f) &&
           (cfg->stalled_rate >= 0.0f) &&
           (cfg->stalled_velocity >= 0.0f) &&
           (cfg->smoothing > 0.0f) && (cfg->smoothing <= 1.0f);
}

/* 功能：初始化 MIT 在线调参器；用途：装载位置刚度和阻尼的调整规则；返回 true 表示初始化成功。 */
bool MotorOnlineMit_Init(motor_online_mit_t *tuner,
                         const motor_online_mit_cfg_t *cfg,
                         bool enabled)
{
    if ((tuner == NULL) || !MotorOnline_MitCfgValid(cfg))
    {
        return false;
    }
    tuner->cfg = *cfg;
    tuner->base_kp = 0.0f;
    tuner->base_kd = 0.0f;
    tuner->applied_kp = 0.0f;
    tuner->applied_kd = 0.0f;
    tuner->enabled = enabled ? 1U : 0U;
    MotorOnlineMit_Reset(tuner);
    return true;
}

/* 功能：设置 MIT 控制的基准 kp、kd；用途：确定在线调整的出发点；返回 true 表示命令在允许范围内。 */
bool MotorOnlineMit_SetCommand(motor_online_mit_t *tuner,
                               float kp,
                               float kd)
{
    if ((tuner == NULL) || !MotorOnline_IsFinite(kp) ||
        !MotorOnline_IsFinite(kd) || (kp < tuner->cfg.minimum_kp) ||
        (kp > tuner->cfg.maximum_kp) || (kd < tuner->cfg.minimum_kd) ||
        (kd > tuner->cfg.maximum_kd))
    {
        return false;
    }
    tuner->base_kp = kp;
    tuner->base_kd = kd;
    tuner->applied_kp = kp;
    tuner->applied_kd = kd;
    MotorOnlineMit_Reset(tuner);
    return true;
}

/* 功能：启用或关闭 MIT 在线调参；用途：切换动态增益与原始命令增益；执行后历史状态被重置。 */
void MotorOnlineMit_SetEnabled(motor_online_mit_t *tuner, bool enabled)
{
    if (tuner == NULL)
    {
        return;
    }
    tuner->enabled = enabled ? 1U : 0U;
    tuner->applied_kp = tuner->base_kp;
    tuner->applied_kd = tuner->base_kd;
    MotorOnlineMit_Reset(tuner);
}

/* 功能：清空 MIT 调参器的误差和收敛历史；用途：开始新的控制过程；执行后下一次更新按首帧处理。 */
void MotorOnlineMit_Reset(motor_online_mit_t *tuner)
{
    if (tuner == NULL)
    {
        return;
    }
    tuner->previous_error = 0.0f;
    tuner->previous_abs_error = 0.0f;
    tuner->convergence_rate = 0.0f;
    tuner->started = 0U;
}

/* 功能：根据位置误差、速度误差和收敛趋势更新 MIT 增益；用途：在线改善响应与阻尼；kp、kd 输出本周期应用值。 */
void MotorOnlineMit_Update(motor_online_mit_t *tuner,
                           float position_error,
                           float velocity_error,
                           float measured_velocity,
                           float dt_s,
                           float *kp,
                           float *kd)
{
    float absolute_error;
    float error_ratio;
    float velocity_ratio;
    float kp_scale;
    float kd_scale;
    float target_kp;
    float target_kd;

    if ((tuner == NULL) || (kp == NULL) || (kd == NULL) ||
        !MotorOnline_IsFinite(position_error) ||
        !MotorOnline_IsFinite(velocity_error) ||
        !MotorOnline_IsFinite(measured_velocity) ||
        !MotorOnline_IsFinite(dt_s) || (dt_s <= 0.0f))
    {
        return;
    }
    if (tuner->enabled == 0U)
    {
        tuner->applied_kp = tuner->base_kp;
        tuner->applied_kd = tuner->base_kd;
        *kp = tuner->applied_kp;
        *kd = tuner->applied_kd;
        return;
    }

    absolute_error = fabsf(position_error);
    if ((absolute_error <= tuner->cfg.near_error) &&
        (fabsf(measured_velocity) <= tuner->cfg.stalled_velocity))
    {
        tuner->applied_kp = tuner->base_kp;
        tuner->applied_kd = tuner->base_kd;
        tuner->previous_error = position_error;
        tuner->previous_abs_error = absolute_error;
        tuner->convergence_rate = 0.0f;
        tuner->started = 1U;
        *kp = tuner->applied_kp;
        *kd = tuner->applied_kd;
        return;
    }
    error_ratio = MotorOnline_Clamp(
        (absolute_error - tuner->cfg.near_error) /
        (tuner->cfg.far_error - tuner->cfg.near_error),
        0.0f,
        1.0f);
    velocity_ratio = MotorOnline_Clamp(
        fabsf(velocity_error) / tuner->cfg.velocity_scale, 0.0f, 1.0f);
    kp_scale = 1.0f + 0.30f * error_ratio;
    kd_scale = 1.0f + 0.20f * (1.0f - error_ratio) +
               0.25f * velocity_ratio;

    if (tuner->started != 0U)
    {
        float instantaneous_rate;

        instantaneous_rate =
            (tuner->previous_abs_error - absolute_error) / dt_s;
        tuner->convergence_rate =
            0.8f * tuner->convergence_rate + 0.2f * instantaneous_rate;
        if ((position_error * tuner->previous_error) < 0.0f)
        {
            kp_scale *= 0.75f;
            kd_scale *= 1.35f;
        }
        else if (tuner->convergence_rate < tuner->cfg.diverging_rate)
        {
            kp_scale *= 0.85f;
            kd_scale *= 1.25f;
        }
        else if ((absolute_error > tuner->cfg.near_error) &&
                 (fabsf(tuner->convergence_rate) <
                  tuner->cfg.stalled_rate) &&
                 (fabsf(measured_velocity) < tuner->cfg.stalled_velocity))
        {
            kp_scale *= 1.12f;
        }
    }
    if ((absolute_error <= tuner->cfg.near_error) &&
        (fabsf(measured_velocity) > tuner->cfg.stalled_velocity))
    {
        kp_scale *= 0.85f;
        kd_scale *= 1.15f;
    }

    kp_scale = MotorOnline_Clamp(kp_scale, 0.65f, 1.40f);
    kd_scale = MotorOnline_Clamp(kd_scale, 0.80f, 1.80f);
    target_kp = MotorOnline_Clamp(tuner->base_kp * kp_scale,
                                  tuner->cfg.minimum_kp,
                                  tuner->cfg.maximum_kp);
    target_kd = MotorOnline_Clamp(tuner->base_kd * kd_scale,
                                  tuner->cfg.minimum_kd,
                                  tuner->cfg.maximum_kd);
    tuner->applied_kp = MotorOnline_Approach(
        tuner->applied_kp, target_kp, tuner->cfg.smoothing);
    tuner->applied_kd = MotorOnline_Approach(
        tuner->applied_kd, target_kd, tuner->cfg.smoothing);
    tuner->previous_error = position_error;
    tuner->previous_abs_error = absolute_error;
    tuner->started = 1U;
    *kp = tuner->applied_kp;
    *kd = tuner->applied_kd;
}
