#ifndef RAW_UART_BRIDGE_H
#define RAW_UART_BRIDGE_H

#include "stm32h7xx_hal.h"

#include <stdint.h>

extern volatile uint32_t raw_uart_bridge_rx_bytes;
extern volatile uint32_t raw_uart_bridge_tx_bytes;
extern volatile uint32_t raw_uart_bridge_dropped_bytes;
extern volatile uint32_t raw_uart_bridge_rx_error_count;
extern volatile uint32_t raw_uart_bridge_tx_error_count;
extern volatile uint32_t raw_uart_bridge_queue_high_water;

/** Initialize the byte-for-byte USART6 RX to UART7 TX bridge. */
HAL_StatusTypeDef RawUartBridge_Init(UART_HandleTypeDef *rx_uart,
                                     UART_HandleTypeDef *tx_uart);

/** Move received bytes to the TX queue and start pending DMA transfers. */
void RawUartBridge_Run(void);

void RawUartBridge_HandleTxCplt(UART_HandleTypeDef *uart);
void RawUartBridge_HandleUartError(UART_HandleTypeDef *uart);

#endif
