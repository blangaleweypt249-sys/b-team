#ifndef COMM_RUNTIME_H
#define COMM_RUNTIME_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"
#include "cmsis_os2.h"
#include "stm32h7xx_hal.h"

#define COMM_EVENT_UART4_RX  (1UL << 0)
#define COMM_EVENT_USART1_RX (1UL << 1)
#define COMM_EVENT_USART2_RX (1UL << 2)
#define COMM_EVENT_FDCAN1_RX (1UL << 3)
#define COMM_EVENT_FDCAN2_RX (1UL << 4)
#define COMM_EVENT_FDCAN3_RX (1UL << 5)
#define COMM_EVENT_ALL       (COMM_EVENT_UART4_RX | COMM_EVENT_USART1_RX | \
                              COMM_EVENT_USART2_RX | COMM_EVENT_FDCAN1_RX | \
                              COMM_EVENT_FDCAN2_RX | COMM_EVENT_FDCAN3_RX)

typedef enum
{
    COMM_UART_PC,
    COMM_UART_RS485_1,
    COMM_UART_RS485_2
} comm_uart_channel_t;

typedef void (*comm_uart_handler_t)(comm_uart_channel_t channel,
                                    const uint8_t *data,
                                    size_t size,
                                    void *user_data);
typedef void (*comm_can_handler_t)(uint8_t can_bus,
                                   const can_frame_t *frame,
                                   void *user_data);

HAL_StatusTypeDef CommRuntime_Init(osThreadId_t notify_task);
void CommRuntime_Process(uint32_t flags);
void CommRuntime_SetHandlers(comm_uart_handler_t uart_handler,
                             comm_can_handler_t can_handler,
                             void *user_data);
bool CommRuntime_PcTxReady(void);
bool CommRuntime_PcTransmit(const uint8_t *data, uint16_t size);
uint32_t CommRuntime_GetTickMs(void);

void DmaCache_PrepareTx(const void *buffer, size_t size);
void DmaCache_PrepareRx(void *buffer, size_t size);
void DmaCache_CompleteRx(void *buffer, size_t size);

extern volatile uint32_t comm_uart_rx_bytes[3];
extern volatile uint32_t comm_uart_overrun_count[3];
extern volatile uint32_t comm_uart_error_count[3];
extern volatile uint32_t comm_fdcan_rx_count[3];
extern volatile uint32_t comm_notify_error_count;

#ifdef __cplusplus
}
#endif

#endif
