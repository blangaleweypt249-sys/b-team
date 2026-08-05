#ifndef UPPER_ENTRY_H
#define UPPER_ENTRY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "can_frame.h"

bool UpperEntry_Init(void);
void UpperEntry_Control1ms(uint32_t tick_ms);
void UpperEntry_OnPcData(const uint8_t *data,
                         size_t size,
                         uint32_t tick_ms);
void UpperEntry_OnCanFrame(uint8_t can_bus,
                           const can_frame_t *frame,
                           uint32_t tick_ms);

#endif
