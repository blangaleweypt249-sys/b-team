#ifndef J4310_H
#define J4310_H

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

#define J4310_MAX_MOTOR_COUNT  16U

typedef struct
{
    float position_max_rad;
    float velocity_max_rad_s;
    float torque_max_nm;
} j4310_limits_t;

typedef struct
{
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    uint8_t mos_temperature_c;
    uint8_t rotor_temperature_c;
    uint8_t fault;
    uint32_t updated_at_ms;
    uint32_t rx_frames;
} j4310_feedback_t;

typedef struct
{
    bool enabled;
    float base_kp;
    float base_kd;
    float applied_kp;
    float applied_kd;
} j4310_online_mit_state_t;

void J4310_Init(void);
bool J4310_AddMotor(uint8_t motor_id,
                    uint16_t master_id,
                    uint8_t feedback_id,
                    const j4310_limits_t *limits);
bool J4310_BuildEnable(uint8_t motor_id, can_frame_t *frame);
bool J4310_BuildDisable(uint8_t motor_id, can_frame_t *frame);
bool J4310_BuildClearFault(uint8_t motor_id, can_frame_t *frame);
bool J4310_BuildSaveZero(uint8_t motor_id, can_frame_t *frame);
bool J4310_BuildMit(uint8_t motor_id,
                    float position_rad,
                    float velocity_rad_s,
                    float kp,
                    float kd,
                    float torque_nm,
                    can_frame_t *frame);
bool J4310_OnFrame(const can_frame_t *frame, uint32_t tick_ms);
bool J4310_GetFeedback(uint8_t motor_id, j4310_feedback_t *feedback);
bool J4310_SetOnlineMitEnabled(uint8_t motor_id, bool enabled);
bool J4310_GetOnlineMitState(uint8_t motor_id,
                             j4310_online_mit_state_t *state);

#endif
