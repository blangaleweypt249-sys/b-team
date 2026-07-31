#ifndef UPPER_MOTOR_PORT_H
#define UPPER_MOTOR_PORT_H

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"
#include "motor_manager.h"

bool UpperMotorPort_Send(const motor_cfg_t *cfg,
                         const motor_cmd_t *cmd,
                         void *user_data);
void UpperMotorPort_OnFrame(uint8_t can_bus, const can_frame_t *frame);

#endif
