#ifndef DT35_LINK_H
#define DT35_LINK_H

#include "stm32h7xx_hal.h"

/* UART 总线上两个 DT35 传感器的固定地址。 */
#define DT35_LINK_ADDR_41 0x41U
#define DT35_LINK_ADDR_40 0x40U
/* Frame: AA, address, distance_cm low byte, high byte, XOR checksum. */

extern volatile uint16_t dt35_distance_40_cm;
extern volatile uint16_t dt35_distance_41_cm;
extern volatile uint8_t dt35_online_40;
extern volatile uint8_t dt35_online_41;

HAL_StatusTypeDef DT35Link_Init(UART_HandleTypeDef *uart);
void DT35Link_Run(void);
void DT35Link_Send(UART_HandleTypeDef *uart);
void DT35Link_RxCplt(UART_HandleTypeDef *uart);
void DT35Link_Error(UART_HandleTypeDef *uart);

#endif
