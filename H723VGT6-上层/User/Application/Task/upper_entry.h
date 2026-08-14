#ifndef UPPER_ENTRY_H
#define UPPER_ENTRY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "can_frame.h"

extern volatile uint32_t upper_handshake_ack_sent_count;
extern volatile uint32_t upper_handshake_ack_busy_count;
extern volatile uint32_t upper_handshake_ack_fail_count;
extern volatile uint32_t upper_state_sent_count;
extern volatile uint32_t upper_state_busy_count;
extern volatile uint32_t upper_state_fail_count;
extern volatile uint32_t upper_dji_telemetry_sent_count;
extern volatile uint32_t upper_dji_telemetry_busy_count;
extern volatile uint32_t upper_dji_telemetry_fail_count;

bool UpperEntry_Init(void);
void UpperEntry_Control1ms(uint32_t tick_ms);
void UpperEntry_OnPcData(const uint8_t *data,
                         size_t size,
                         uint32_t tick_ms);
void UpperEntry_OnCanFrame(uint8_t can_bus,
                           const can_frame_t *frame,
                           uint32_t tick_ms);

#endif
