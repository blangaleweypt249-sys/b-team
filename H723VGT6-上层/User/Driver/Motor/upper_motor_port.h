#ifndef UPPER_MOTOR_PORT_H
#define UPPER_MOTOR_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "can_frame.h"
#include "motor_manager.h"

typedef struct
{
    uint32_t active_mask;
    uint32_t offline_mask;
    uint32_t fault_mask;
    uint32_t protocol_block_mask;
} upper_motor_health_t;

typedef struct
{
    motor_model_t model;
    uint8_t can_bus;
    uint8_t node_id;
    uint8_t error_code;
    uint32_t tick_ms;
    uint32_t sequence;
} upper_motor_fault_t;

typedef struct
{
    motor_model_t model;
    uint8_t can_bus;
    uint8_t node_id;
    bool feedback_received;
    bool zero_valid;
    bool feedback_fresh;
    float rotor_position_rad;
    float zero_rotor_position_rad;
    float relative_output_position_rad;
} upper_dji_diagnostic_t;

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
    uint8_t last_result;
} upper_j4310_rx_diagnostic_t;

typedef struct
{
    uint32_t attempted_frames;
    uint32_t queued_frames;
    uint32_t failed_frames;
    uint32_t enable_frames;
    uint32_t mit_frames;
    uint32_t disable_frames;
    uint16_t last_can_id;
    uint8_t last_dlc;
    uint8_t last_data7;
    bool enable_confirmed;
    uint8_t feedback_state;
} upper_j4310_tx_diagnostic_t;

typedef struct
{
    float position_rad;
    float velocity_rad_s;
    uint8_t state;
} upper_j4310_feedback_t;

typedef struct
{
    float position_rad;
    float velocity_rad_s;
} upper_motor_feedback_t;

#define UPPER_MOTOR_ERROR_FEEDBACK_TIMEOUT 0xF0U

bool UpperMotorPort_Init(const motor_cfg_t *cfg, size_t motor_count);
void UpperMotorPort_BeginCycle(uint32_t tick_ms);
bool UpperMotorPort_Send(const motor_cfg_t *cfg,
                         const motor_cmd_t *cmd,
                         void *user_data);
bool UpperMotorPort_Flush(void);
void UpperMotorPort_OnFrame(uint8_t can_bus,
                             const can_frame_t *frame,
                             uint32_t tick_ms);
bool UpperMotorPort_GetHealth(uint32_t tick_ms,
                              upper_motor_health_t *health);
bool UpperMotorPort_SaveJ4310Zero(uint8_t can_bus, uint8_t node_id);
bool UpperMotorPort_GetJ4310OutputPosition(uint8_t can_bus,
                                           uint8_t node_id,
                                           float *position_rad);
bool UpperMotorPort_GetJ4310Feedback(uint8_t can_bus,
                                     uint8_t node_id,
                                     upper_j4310_feedback_t *feedback);
bool UpperMotorPort_GetMotorFeedback(size_t motor_index,
                                     upper_motor_feedback_t *feedback);
bool UpperMotorPort_GetJ4310RxDiagnostic(
    uint8_t can_bus,
    uint8_t node_id,
    upper_j4310_rx_diagnostic_t *diagnostic);
bool UpperMotorPort_GetJ4310TxDiagnostic(
    uint8_t can_bus,
    uint8_t node_id,
    upper_j4310_tx_diagnostic_t *diagnostic);
size_t UpperMotorPort_GetDjiDiagnostics(uint32_t tick_ms,
                                        upper_dji_diagnostic_t *diagnostics,
                                        size_t capacity);
void UpperMotorPort_RecordExternalFault(const motor_cfg_t *cfg,
                                        uint8_t error_code,
                                        uint32_t tick_ms);
bool UpperMotorPort_GetPendingFault(upper_motor_fault_t *fault);
void UpperMotorPort_MarkFaultSent(uint32_t sequence);
bool UpperMotorPort_IsDjiConfigured(uint8_t can_bus,
                                    motor_model_t model,
                                    uint8_t node_id);

#endif
