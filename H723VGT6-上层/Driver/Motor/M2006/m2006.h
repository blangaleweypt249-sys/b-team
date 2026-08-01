#ifndef M2006_H
#define M2006_H

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

#define M2006_CAN_BUS_COUNT  3U
#define M2006_MOTOR_COUNT    8U

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
    float current_limit_a;
    float position_vel_limit_rad_s;
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
bool M2006_GetTimeoutStats(uint8_t can_bus,
                           uint8_t motor_id,
                           m2006_timeout_stats_t *stats);

#endif
