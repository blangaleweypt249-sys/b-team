#ifndef VOFA_BRIDGE_H
#define VOFA_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "can_frame.h"

void VofaBridge_Init(void);
bool VofaBridge_Receive(const uint8_t *data,
                        size_t size,
                        uint32_t tick_ms);
void VofaBridge_Control1ms(uint32_t tick_ms);
void VofaBridge_OnCanFrame(uint8_t can_bus,
                           const can_frame_t *frame,
                           uint32_t tick_ms);
bool VofaBridge_IsActive(void);

#endif
