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

#endif
