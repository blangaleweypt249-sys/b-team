#ifndef REMOTE_LINK_H
#define REMOTE_LINK_H

#include "main.h"

extern volatile uint32_t remote_link_uart_errors;
extern volatile uint32_t remote_link_rx_bytes;
extern volatile uint32_t remote_link_rx_callbacks;
extern volatile uint32_t remote_link_rx_arm_errors;
extern volatile uint32_t remote_link_forward_errors;
extern volatile uint32_t remote_link_forward_overflows;
extern volatile uint32_t remote_link_forwarded_bytes;
extern volatile uint32_t remote_link_control_frames;
extern volatile uint32_t remote_link_switch_events;
extern volatile uint32_t remote_link_aux_control_frames;
extern volatile uint32_t remote_link_aux_crc_errors;
extern volatile uint32_t remote_link_aux_uart_errors;

void RemoteLink_Init(void);
void RemoteLink_ForwardRawData(void);

#endif
