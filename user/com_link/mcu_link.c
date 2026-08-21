#include "mcu_link.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

static UART_HandleTypeDef *mcu_uart;
static uint8_t mcu_tx_buffer[MCU_LINK_TX_BUFFER_SIZE];
static uint8_t mcu_pending_buffer[MCU_LINK_TX_BUFFER_SIZE];
static volatile uint16_t mcu_pending_length;
static volatile bool mcu_pending_valid;
static volatile bool mcu_tx_busy;
static bool mcu_initialized;

volatile uint32_t mcu_link_uart_error_count;
volatile uint32_t mcu_link_tx_error_count;

HAL_StatusTypeDef McuLink_InitTx(UART_HandleTypeDef *uart)
{
    if ((uart == NULL) || (uart->Instance != USART6) ||
        (uart->hdmatx == NULL))
    {
        return HAL_ERROR;
    }
    if (mcu_initialized)
    {
        return (mcu_uart == uart) ? HAL_OK : HAL_ERROR;
    }

    mcu_uart = uart;
    (void)memset(mcu_tx_buffer, 0, sizeof(mcu_tx_buffer));
    (void)memset(mcu_pending_buffer, 0, sizeof(mcu_pending_buffer));
    mcu_pending_length = 0U;
    mcu_pending_valid = false;
    mcu_tx_busy = false;
    mcu_link_uart_error_count = 0U;
    mcu_link_tx_error_count = 0U;
    mcu_initialized = true;
    return HAL_OK;
}

HAL_StatusTypeDef McuLink_Send(const uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status;

    if (!mcu_initialized || (data == NULL) || (length == 0U) ||
        (length > MCU_LINK_TX_BUFFER_SIZE))
    {
        return HAL_ERROR;
    }
    if (mcu_tx_busy || mcu_pending_valid)
    {
        /* The remote control is a periodic state stream. Keep only its newest
         * frame so a slow UART cannot block reception or build a stale queue. */
        (void)memcpy(mcu_pending_buffer, data, length);
        mcu_pending_length = length;
        mcu_pending_valid = true;
        return HAL_OK;
    }

    (void)memcpy(mcu_tx_buffer, data, length);
    mcu_tx_busy = true;
    status = HAL_UART_Transmit_DMA(mcu_uart, mcu_tx_buffer, length);
    if (status != HAL_OK)
    {
        mcu_tx_busy = false;
        mcu_link_tx_error_count++;
    }
    return status;
}

void McuLink_Run(void)
{
    uint16_t length;

    if (!mcu_initialized || mcu_tx_busy || !mcu_pending_valid)
    {
        return;
    }

    __disable_irq();
    if (mcu_tx_busy || !mcu_pending_valid)
    {
        __enable_irq();
        return;
    }
    length = mcu_pending_length;
    (void)memcpy(mcu_tx_buffer, mcu_pending_buffer, length);
    mcu_pending_valid = false;
    mcu_pending_length = 0U;
    mcu_tx_busy = true;
    __enable_irq();

    if (HAL_UART_Transmit_DMA(mcu_uart, mcu_tx_buffer, length) != HAL_OK)
    {
        mcu_tx_busy = false;
        mcu_link_tx_error_count++;
    }
}

void McuLink_HandleTxCplt(UART_HandleTypeDef *uart)
{
    if (mcu_initialized && (uart == mcu_uart))
    {
        mcu_tx_busy = false;
    }
}

void McuLink_HandleUartError(UART_HandleTypeDef *uart)
{
    if (!mcu_initialized || (uart != mcu_uart))
    {
        return;
    }

    mcu_link_uart_error_count++;
    mcu_tx_busy = false;
}
