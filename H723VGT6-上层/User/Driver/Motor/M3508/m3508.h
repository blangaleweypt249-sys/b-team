#ifndef M3508_H
#define M3508_H

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

#define M3508_CAN_BUS_COUNT  3U
#define M3508_MOTOR_COUNT    8U
#define M3508_DEFAULT_ACCEL_LIMIT_RAD_S2 52.35987756f

typedef enum
{
    M3508_MODE_STOP,
    M3508_MODE_CURRENT,
    M3508_MODE_VELOCITY,
    M3508_MODE_POSITION
} m3508_mode_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
} m3508_pid_cfg_t;

typedef struct
{
    bool enabled;
    uint8_t strategy;
    uint8_t active_rule;
    float applied_kp;
    float applied_ki;
    float applied_kd;
} m3508_online_pid_state_t;

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
} m3508_motion_state_t;

typedef enum
{
    M3508_AUTOTUNE_IDLE,
    M3508_AUTOTUNE_RUNNING,
    M3508_AUTOTUNE_COMPLETE,
    M3508_AUTOTUNE_FAILED
} m3508_autotune_status_t;

typedef struct
{
    m3508_autotune_status_t status;
    float relay_current_a;
    float hysteresis_rad_s;
    float oscillation_amplitude_rad_s;
    float ultimate_period_s;
    float ultimate_gain;
    m3508_pid_cfg_t result;
} m3508_autotune_state_t;

typedef struct
{
    float current_limit_a;
    float position_vel_limit_rad_s;
    float acceleration_limit_rad_s2;
    uint32_t feedback_timeout_ms;
    uint32_t command_timeout_ms;
    m3508_pid_cfg_t speed_pid;
    m3508_pid_cfg_t position_pid;
} m3508_cfg_t;

typedef struct
{
    uint8_t can_bus;
    uint8_t motor_id;
    uint16_t rotor_encoder;
    int16_t rotor_speed_rpm;
    int16_t torque_current_raw;
    uint8_t temperature_c;
    int64_t total_encoder_counts;
    int64_t zero_encoder_counts;
    float output_pos_rad;
    float output_vel_rad_s;
    float torque_current_a;
    uint32_t updated_at_ms;
    uint32_t rx_frames;
} m3508_feedback_t;

typedef struct
{
    bool command_timed_out;
    bool feedback_timed_out;
    uint32_t command_timeout_count;
    uint32_t feedback_timeout_count;
} m3508_timeout_stats_t;

bool M3508_Init(const m3508_cfg_t *cfg);
bool M3508_SetTarget(uint8_t can_bus,
                     uint8_t motor_id,
                     m3508_mode_t mode,
                     float target,
                     uint32_t tick_ms);
bool M3508_CalcCurrentRaw(uint8_t can_bus,
                          uint8_t motor_id,
                          uint32_t tick_ms,
                          int16_t *current_raw);
bool M3508_OnFrame(uint8_t can_bus,
                    uint8_t motor_id,
                    const can_frame_t *frame,
                    uint32_t tick_ms);
bool M3508_GetFeedback(uint8_t can_bus,
                       uint8_t motor_id,
                       m3508_feedback_t *feedback);
bool M3508_GetFeedbackSnapshot(uint8_t can_bus,
                               uint8_t motor_id,
                               m3508_feedback_t *feedback,
                               bool *zero_valid);
bool M3508_GetTimeoutStats(uint8_t can_bus,
                           uint8_t motor_id,
                           m3508_timeout_stats_t *stats);
bool M3508_SetSpeedPid(uint8_t can_bus,
                       uint8_t motor_id,
                       const m3508_pid_cfg_t *cfg);
bool M3508_SetPositionPid(uint8_t can_bus,
                          uint8_t motor_id,
                          const m3508_pid_cfg_t *cfg);
bool M3508_SetCurrentLimit(uint8_t can_bus,
                           uint8_t motor_id,
                           float current_limit_a);
bool M3508_SetPositionVelocityLimit(uint8_t can_bus,
                                    uint8_t motor_id,
                                    float velocity_limit_rad_s);
bool M3508_SetAccelerationLimit(uint8_t can_bus,
                                uint8_t motor_id,
                                float acceleration_limit_rad_s2);
bool M3508_SetOnlinePidEnabled(uint8_t can_bus,
                               uint8_t motor_id,
                               bool enabled);
bool M3508_ZeroPosition(uint8_t can_bus, uint8_t motor_id);
bool M3508_GetOnlinePidState(uint8_t can_bus,
                             uint8_t motor_id,
                             m3508_online_pid_state_t *state);
bool M3508_GetMotionState(uint8_t can_bus,
                          uint8_t motor_id,
                          m3508_motion_state_t *state);
bool M3508_GetSpeedPid(uint8_t can_bus,
                       uint8_t motor_id,
                       m3508_pid_cfg_t *cfg);
bool M3508_GetPositionPid(uint8_t can_bus,
                          uint8_t motor_id,
                          m3508_pid_cfg_t *cfg);
bool M3508_StartSpeedAutoTune(uint8_t can_bus,
                              uint8_t motor_id,
                              float relay_current_a,
                              float hysteresis_rad_s,
                              float safety_velocity_rad_s,
                              uint32_t tick_ms);
bool M3508_CancelAutoTune(uint8_t can_bus, uint8_t motor_id);
bool M3508_GetAutoTuneState(uint8_t can_bus,
                            uint8_t motor_id,
                            m3508_autotune_state_t *state);

#endif
