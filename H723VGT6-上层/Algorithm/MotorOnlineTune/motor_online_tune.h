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

bool MotorOnlinePid_Init(motor_online_pid_t *tuner,
                         const motor_online_pid_cfg_t *cfg,
                         bool enabled);
bool MotorOnlinePid_SetBaseGains(motor_online_pid_t *tuner,
                                 motor_online_gains_t gains);
void MotorOnlinePid_SetEnabled(motor_online_pid_t *tuner, bool enabled);
void MotorOnlinePid_Reset(motor_online_pid_t *tuner, bool restore_gains);
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

bool MotorOnlineMit_Init(motor_online_mit_t *tuner,
                         const motor_online_mit_cfg_t *cfg,
                         bool enabled);
bool MotorOnlineMit_SetCommand(motor_online_mit_t *tuner,
                               float kp,
                               float kd);
void MotorOnlineMit_SetEnabled(motor_online_mit_t *tuner, bool enabled);
void MotorOnlineMit_Reset(motor_online_mit_t *tuner);
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
