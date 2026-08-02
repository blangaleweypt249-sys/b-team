#ifndef M3508_H
#define M3508_H

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

#define M3508_MOTOR_COUNT  8U

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
    float current_limit_a;
    float position_vel_limit_rad_s;
    uint32_t feedback_timeout_ms;
    uint32_t command_timeout_ms;
    m3508_pid_cfg_t speed_pid;
    m3508_pid_cfg_t position_pid;
} m3508_cfg_t;

typedef struct
{
    uint8_t motor_id;
    uint16_t rotor_encoder;
    int16_t rotor_speed_rpm;
    int16_t torque_current_raw;
    uint8_t temperature_c;
    int64_t total_encoder_counts;
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
bool M3508_SetTarget(uint8_t motor_id,
                     m3508_mode_t mode,
                     float target,
                     uint32_t tick_ms);
bool M3508_CalcCurrentRaw(uint8_t motor_id,
                          uint32_t tick_ms,
                          int16_t *current_raw);
bool M3508_OnFrame(uint8_t motor_id,
                    const can_frame_t *frame,
                    uint32_t tick_ms);
bool M3508_GetFeedback(uint8_t motor_id, m3508_feedback_t *feedback);
bool M3508_GetTimeoutStats(uint8_t motor_id,
                           m3508_timeout_stats_t *stats);
bool M3508_SetSpeedPid(uint8_t motor_id,
                       const m3508_pid_cfg_t *cfg);
bool M3508_SetOnlinePidEnabled(uint8_t motor_id, bool enabled);
bool M3508_GetOnlinePidState(uint8_t motor_id,
                             m3508_online_pid_state_t *state);

#endif
