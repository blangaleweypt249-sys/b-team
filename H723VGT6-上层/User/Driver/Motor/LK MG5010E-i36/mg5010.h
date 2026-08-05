#ifndef MG5010_H
#define MG5010_H

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"
#include "motor_online_tune.h"

#define MG5010_MOTOR_ID_MIN  1U
#define MG5010_MOTOR_ID_MAX  32U

typedef struct
{
    uint8_t motor_id;
    uint8_t command;
    int8_t temperature_c;
    uint8_t motor_state;
    uint8_t error_state;
    float bus_voltage_v;
    float bus_current_a;
    float torque_current_a;
    float output_vel_rad_s;
    float output_pos_rad;
    uint16_t encoder;
    uint32_t updated_at_ms;
    uint32_t rx_frames;
    bool output_pos_valid;
} mg5010_feedback_t;

typedef struct
{
    bool enabled;
    uint8_t strategy;
    uint8_t active_rule;
    float target_vel_rad_s;
    float measured_vel_rad_s;
    float speed_error_rad_s;
    float current_command_a;
    float current_limit_a;
    motor_online_gains_t base_gains;
    motor_online_gains_t applied_gains;
} mg5010_online_pid_state_t;

void Mg5010_Init(void);
bool Mg5010_BuildRun(uint8_t motor_id, can_frame_t *frame);
bool Mg5010_BuildStop(uint8_t motor_id, can_frame_t *frame);
bool Mg5010_BuildReadPosition(uint8_t motor_id, can_frame_t *frame);
bool Mg5010_BuildCurrent(uint8_t motor_id,
                         float current_a,
                         can_frame_t *frame);
bool Mg5010_BuildVelocity(uint8_t motor_id,
                          float output_vel_rad_s,
                          float current_limit_a,
                          can_frame_t *frame);
bool Mg5010_BuildControlledVelocity(uint8_t motor_id,
                                    float output_vel_rad_s,
                                    float current_limit_a,
                                    float dt_s,
                                    can_frame_t *frame);
bool Mg5010_BuildPosition(uint8_t motor_id,
                          float output_pos_rad,
                          float max_output_vel_rad_s,
                          can_frame_t *frame);
bool Mg5010_OnFrame(uint8_t motor_id,
                     const can_frame_t *frame,
                     uint32_t tick_ms);
bool Mg5010_PositionReady(uint8_t motor_id);
bool Mg5010_GetFeedback(uint8_t motor_id, mg5010_feedback_t *feedback);
bool Mg5010_SetOuterSpeedPid(uint8_t motor_id,
                             motor_online_gains_t gains,
                             float integral_limit);
bool Mg5010_SetOnlinePidEnabled(uint8_t motor_id, bool enabled);
bool Mg5010_GetOnlinePidState(uint8_t motor_id,
                              mg5010_online_pid_state_t *state);

#endif
