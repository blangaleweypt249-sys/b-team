#ifndef J4310_H
#define J4310_H

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

#define J4310_MAX_MOTOR_COUNT  16U

typedef enum
{
    J4310_MODE_MIT = 0x000U,
    J4310_MODE_POSITION_VELOCITY = 0x100U,
    J4310_MODE_VELOCITY = 0x200U
} j4310_mode_t;

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

typedef enum
{
    J4310_RX_NONE = 0U,
    J4310_RX_ACCEPTED = 1U,
    J4310_RX_REJECTED_FORMAT = 2U,
    J4310_RX_REJECTED_MASTER_ID = 3U,
    J4310_RX_REJECTED_FEEDBACK_ID = 4U
} j4310_rx_result_t;

typedef struct
{
    uint32_t frames_seen;
    uint32_t accepted_frames;
    uint32_t rejected_format_frames;
    uint32_t rejected_master_id_frames;
    uint32_t rejected_feedback_id_frames;
    uint16_t last_can_id;
    uint8_t last_dlc;
    uint8_t last_data0;
    j4310_rx_result_t last_result;
} j4310_rx_diagnostics_t;

void J4310_Init(void);
bool J4310_AddMotor(uint8_t motor_id,
                    uint16_t master_id,
                    uint8_t feedback_id,
                    j4310_mode_t mode,
                    const j4310_limits_t *limits);
bool J4310_SetMode(uint8_t motor_id, j4310_mode_t mode);
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
bool J4310_BuildPositionVelocity(uint8_t motor_id,
                                 float position_rad,
                                 float velocity_rad_s,
                                 can_frame_t *frame);
bool J4310_BuildVelocity(uint8_t motor_id,
                         float velocity_rad_s,
                         can_frame_t *frame);
bool J4310_SetTorqueLimit(uint8_t motor_id, float torque_limit_nm);
bool J4310_OnFrame(const can_frame_t *frame, uint32_t tick_ms);
bool J4310_GetFeedback(uint8_t motor_id, j4310_feedback_t *feedback);
bool J4310_GetRxDiagnostics(j4310_rx_diagnostics_t *diagnostics);
bool J4310_SetOnlineMitEnabled(uint8_t motor_id, bool enabled);
bool J4310_GetOnlineMitState(uint8_t motor_id,
                             j4310_online_mit_state_t *state);

#endif
