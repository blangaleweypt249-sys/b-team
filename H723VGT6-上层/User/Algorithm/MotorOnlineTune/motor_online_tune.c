#include "motor_online_tune.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

static bool MotorOnline_IsFinite(float value)
{
    return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX);
}

static float MotorOnline_Clamp(float value, float min, float max)
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

static float MotorOnline_Approach(float current, float target, float alpha)
{
    return current + (target - current) * alpha;
}

static bool MotorOnline_GainsFinite(motor_online_gains_t gains)
{
    return MotorOnline_IsFinite(gains.kp) &&
           MotorOnline_IsFinite(gains.ki) &&
           MotorOnline_IsFinite(gains.kd);
}

static motor_online_gains_t MotorOnline_ClampGains(
    motor_online_gains_t gains,
    const motor_online_pid_cfg_t *cfg)
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

static bool MotorOnline_PidCfgValid(const motor_online_pid_cfg_t *cfg)
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

void MotorOnlinePid_SetEnabled(motor_online_pid_t *tuner, bool enabled)
{
    if (tuner == NULL)
    {
        return;
    }
    tuner->enabled = enabled ? 1U : 0U;
    MotorOnlinePid_Reset(tuner, true);
}

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

static void MotorOnline_Adapt(motor_online_pid_t *tuner,
                              float error,
                              float error_rate,
                              float dt_s)
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

static motor_online_gains_t MotorOnline_ApplyExpert(
    motor_online_pid_t *tuner,
    motor_online_gains_t source,
    float error,
    float error_rate)
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

static bool MotorOnline_MitCfgValid(const motor_online_mit_cfg_t *cfg)
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
