#ifndef USART_H
#define USART_H

#include "main.h"

/*
 * USART1 console (STM32F405RGT6)
 * --------------------------------
 * Pins:   PA9 = TX, PA10 = RX  (AF7, USART1 default mapping)
 * Format: 8 data bits, no parity, 1 stop bit (8N1)
 * Baud:   115200 (divider derived from the configured APB2 clock at runtime,
 *         so the clock configuration is NOT modified)
 * RX:     interrupt-driven, one ASCII line at a time (terminated by '\n')
 *
 * NOTE: This peripheral is hand-written (register level) because CubeMX had
 *       only copied the HAL driver files for CAN. The .ioc file was intentionally
 *       NOT edited to avoid corrupting CubeMX generation; if you regenerate the
 *       project with CubeMX, enable USART1 (PA9/PA10) in the GUI too.
 */

#define USART1_BAUDRATE       115200U   /* 84000000 / 115200 -> BRR 729 (OVER16) */
#define USART1_RX_LINE_MAX    64U       /* max length of one command line, excl. NUL */

/* Initialize USART1 (clock, GPIO PA9/PA10 AF7, 8N1, NVIC) but do not start RX. */
void MX_USART1_Init(void);

/* Enable the RXNE interrupt (begin receiving). Call once after MX_USART1_Init(). */
void usart1_start_rx(void);

/* Blocking transmit of a NUL-terminated string. Used for command ack/error text. */
void usart1_puts(const char *s);

/*
 * Non-blocking: if a full line has been received, copy it into 'out' (at most
 * maxlen-1 chars, NUL-terminated) and return 1. Returns 0 if no complete line
 * is available. After a successful copy the receive buffer is reset for the next
 * line; bytes that arrive while a line is pending are discarded.
 */
int usart1_getline(char *out, uint16_t maxlen);

/* ---- USART6: IMU over RS485 (HAL-based, for the imported imu driver) ----
 * PC6=TX, PC7=RX (AF8), 921600 8N1, RX via DMA2 Stream1 / Ch5 (circular).
 * RS485 direction (DE/RE) is handled by the transceiver hardware. */
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef  hdma_usart6_rx;
void MX_USART6_UART_Init(void);

#endif /* USART_H */
