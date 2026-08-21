#ifndef MCU_LINK_H
#define MCU_LINK_H

#include "stm32h7xx_hal.h"

#include <stdint.h>

#define MCU_LINK_TX_BUFFER_SIZE 512U

extern volatile uint32_t mcu_link_uart_error_count;
extern volatile uint32_t mcu_link_tx_error_count;

/**
 * @brief 仅初始化 USART6 发送，用于把遥控帧转发给第二主控
 * @param uart USART6 句柄
 * @retval HAL 状态
 */
HAL_StatusTypeDef McuLink_InitTx(UART_HandleTypeDef *uart);

/**
 * @brief 通过 USART6 DMA 发送数据
 * @param data 待发送数据
 * @param length 数据长度，不得超过 MCU_LINK_TX_BUFFER_SIZE
 * @retval HAL 状态
 */
HAL_StatusTypeDef McuLink_Send(const uint8_t *data, uint16_t length);

/** Start a pending latest-frame transfer without blocking the caller. */
void McuLink_Run(void);

void McuLink_HandleTxCplt(UART_HandleTypeDef *uart);
void McuLink_HandleUartError(UART_HandleTypeDef *uart);

#endif
