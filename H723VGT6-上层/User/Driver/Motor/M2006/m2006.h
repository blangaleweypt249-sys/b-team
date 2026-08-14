#ifndef M2006_H
#define M2006_H

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

#define M2006_CAN_BUS_COUNT  3U
#define M2006_MOTOR_COUNT    8U
#define M2006_DEFAULT_ACCEL_LIMIT_RAD_S2 31.41592654f

typedef enum
{
    M2006_MODE_STOP,
    M2006_MODE_CURRENT,
    M2006_MODE_VELOCITY,
    M2006_MODE_POSITION
} m2006_mode_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
} m2006_pid_cfg_t;

typedef struct
{
    bool enabled;
    uint8_t strategy;
    uint8_t active_rule;
    float applied_kp;
    float applied_ki;
    float applied_kd;
} m2006_online_pid_state_t;

typedef struct
{
    float final_position_rad;
    float reference_position_rad;
    float final_velocity_rad_s;
    float reference_velocity_rad_s;
    float reference_acceleration_rad_s2;
    float current_command_a;
    float position_error_rad;
    float velocity_error_rad_s;
    float trajectory_progress;
    bool trajectory_active;
} m2006_motion_state_t;

typedef enum
{
    M2006_AUTOTUNE_IDLE,
    M2006_AUTOTUNE_RUNNING,
    M2006_AUTOTUNE_COMPLETE,
    M2006_AUTOTUNE_FAILED
} m2006_autotune_status_t;

typedef struct
{
    m2006_autotune_status_t status;
    float relay_current_a;
    float hysteresis_rad_s;
    float oscillation_amplitude_rad_s;
    float ultimate_period_s;
    float ultimate_gain;
    m2006_pid_cfg_t result;
} m2006_autotune_state_t;

typedef struct
{
    float current_limit_a;
    float position_vel_limit_rad_s;
    float acceleration_limit_rad_s2;
    uint32_t feedback_timeout_ms;
    uint32_t command_timeout_ms;
    m2006_pid_cfg_t speed_pid;
    m2006_pid_cfg_t position_pid;
} m2006_cfg_t;

typedef struct
{
    uint8_t can_bus;
    uint8_t motor_id;
    uint16_t rotor_encoder;
    int16_t rotor_speed_rpm;
    int16_t torque_current_raw;
    int64_t total_encoder_counts;
    int64_t zero_encoder_counts;
    float output_pos_rad;
    float output_vel_rad_s;
    float torque_current_a;
    uint32_t updated_at_ms;
    uint32_t rx_frames;
} m2006_feedback_t;

typedef struct
{
    bool command_timed_out;
    bool feedback_timed_out;
    uint32_t command_timeout_count;
    uint32_t feedback_timeout_count;
} m2006_timeout_stats_t;

bool M2006_Init(const m2006_cfg_t *cfg);
bool M2006_SetTarget(uint8_t can_bus,
                     uint8_t motor_id,
                     m2006_mode_t mode,
                     float target,
                     uint32_t tick_ms);
bool M2006_CalcCurrentRaw(uint8_t can_bus,
                          uint8_t motor_id,
                          uint32_t tick_ms,
                          int16_t *current_raw);
bool M2006_OnFrame(uint8_t can_bus,
                    uint8_t motor_id,
                    const can_frame_t *frame,
                    uint32_t tick_ms);
bool M2006_GetFeedback(uint8_t can_bus,
                       uint8_t motor_id,
                       m2006_feedback_t *feedback);
bool M2006_GetFeedbackSnapshot(uint8_t can_bus,
                               uint8_t motor_id,
                               m2006_feedback_t *feedback,
                               bool *zero_valid);
bool M2006_GetTimeoutStats(uint8_t can_bus,
                           uint8_t motor_id,
                           m2006_timeout_stats_t *stats);
bool M2006_SetSpeedPid(uint8_t can_bus,
                       uint8_t motor_id,
                       const m2006_pid_cfg_t *cfg);
bool M2006_SetPositionPid(uint8_t can_bus,
                          uint8_t motor_id,
                          const m2006_pid_cfg_t *cfg);
bool M2006_SetCurrentLimit(uint8_t can_bus,
                           uint8_t motor_id,
                           float current_limit_a);
bool M2006_SetPositionVelocityLimit(uint8_t can_bus,
                                    uint8_t motor_id,
                                    float velocity_limit_rad_s);
bool M2006_SetAccelerationLimit(uint8_t can_bus,
                                uint8_t motor_id,
                                float acceleration_limit_rad_s2);
bool M2006_SetOnlinePidEnabled(uint8_t can_bus,
                               uint8_t motor_id,
                               bool enabled);
bool M2006_ZeroPosition(uint8_t can_bus, uint8_t motor_id);
bool M2006_GetOnlinePidState(uint8_t can_bus,
                             uint8_t motor_id,
                             m2006_online_pid_state_t *state);
bool M2006_GetMotionState(uint8_t can_bus,
                          uint8_t motor_id,
                          m2006_motion_state_t *state);
bool M2006_GetSpeedPid(uint8_t can_bus,
                       uint8_t motor_id,
                       m2006_pid_cfg_t *cfg);
bool M2006_GetPositionPid(uint8_t can_bus,
                          uint8_t motor_id,
                          m2006_pid_cfg_t *cfg);
bool M2006_StartSpeedAutoTune(uint8_t can_bus,
                              uint8_t motor_id,
                              float relay_current_a,
                              float hysteresis_rad_s,
                              float safety_velocity_rad_s,
                              uint32_t tick_ms);
bool M2006_CancelAutoTune(uint8_t can_bus, uint8_t motor_id);
bool M2006_GetAutoTuneState(uint8_t can_bus,
                            uint8_t motor_id,
                            m2006_autotune_state_t *state);

#endif
