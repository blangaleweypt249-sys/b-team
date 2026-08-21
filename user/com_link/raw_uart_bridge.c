#include "raw_uart_bridge.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define RAW_BRIDGE_RX_DMA_BUFFER_SIZE 2048U
#define RAW_BRIDGE_TX_QUEUE_SIZE      32768U
#define RAW_BRIDGE_PACKET_SIZE        8U

static UART_HandleTypeDef *bridge_rx_uart;
static UART_HandleTypeDef *bridge_tx_uart;
static uint8_t bridge_rx_dma_buffer[RAW_BRIDGE_RX_DMA_BUFFER_SIZE];
static uint8_t bridge_tx_queue[RAW_BRIDGE_TX_QUEUE_SIZE];
static uint16_t bridge_rx_read_pos;
static uint16_t bridge_tx_read_pos;
static uint16_t bridge_tx_write_pos;
static uint16_t bridge_tx_queued;
static uint16_t bridge_tx_transfer_length;
static volatile bool bridge_tx_complete;
static volatile bool bridge_rx_restart_requested;
static volatile bool bridge_tx_restart_requested;
static bool bridge_tx_busy;
static bool bridge_initialized;

volatile uint32_t raw_uart_bridge_rx_bytes;
volatile uint32_t raw_uart_bridge_tx_bytes;
volatile uint32_t raw_uart_bridge_dropped_bytes;
volatile uint32_t raw_uart_bridge_rx_error_count;
volatile uint32_t raw_uart_bridge_tx_error_count;
volatile uint32_t raw_uart_bridge_queue_high_water;

static HAL_StatusTypeDef RawUartBridge_StartReceive(void)
{
    HAL_StatusTypeDef status;

    bridge_rx_read_pos = 0U;
    __HAL_UART_CLEAR_FLAG(bridge_rx_uart,
                          UART_CLEAR_OREF | UART_CLEAR_NEF |
                          UART_CLEAR_PEF | UART_CLEAR_FEF);
    __HAL_UART_SEND_REQ(bridge_rx_uart, UART_RXDATA_FLUSH_REQUEST);
    status = HAL_UART_Receive_DMA(bridge_rx_uart, bridge_rx_dma_buffer,
                                  RAW_BRIDGE_RX_DMA_BUFFER_SIZE);
    if (status == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(bridge_rx_uart->hdmarx, DMA_IT_HT | DMA_IT_TC);
    }
    return status;
}

static void RawUartBridge_FinishTransmit(void)
{
    if (!bridge_tx_complete)
    {
        return;
    }

    bridge_tx_complete = false;
    bridge_tx_read_pos = (uint16_t)(bridge_tx_read_pos +
                                    bridge_tx_transfer_length);
    if (bridge_tx_read_pos >= RAW_BRIDGE_TX_QUEUE_SIZE)
    {
        bridge_tx_read_pos = 0U;
    }
    bridge_tx_queued = (uint16_t)(bridge_tx_queued -
                                  bridge_tx_transfer_length);
    raw_uart_bridge_tx_bytes += bridge_tx_transfer_length;
    bridge_tx_transfer_length = 0U;
    bridge_tx_busy = false;
}

static void RawUartBridge_QueueReceived(void)
{
    uint16_t write_pos;

    write_pos = (uint16_t)(RAW_BRIDGE_RX_DMA_BUFFER_SIZE -
                           __HAL_DMA_GET_COUNTER(bridge_rx_uart->hdmarx));
    if (write_pos >= RAW_BRIDGE_RX_DMA_BUFFER_SIZE)
    {
        write_pos = 0U;
    }
    __DMB();

    while (bridge_rx_read_pos != write_pos)
    {
        uint8_t data = bridge_rx_dma_buffer[bridge_rx_read_pos];

        bridge_rx_read_pos++;
        if (bridge_rx_read_pos >= RAW_BRIDGE_RX_DMA_BUFFER_SIZE)
        {
            bridge_rx_read_pos = 0U;
        }
        raw_uart_bridge_rx_bytes++;

        if (bridge_tx_queued >= RAW_BRIDGE_TX_QUEUE_SIZE)
        {
            raw_uart_bridge_dropped_bytes++;
            continue;
        }

        bridge_tx_queue[bridge_tx_write_pos] = data;
        bridge_tx_write_pos++;
        if (bridge_tx_write_pos >= RAW_BRIDGE_TX_QUEUE_SIZE)
        {
            bridge_tx_write_pos = 0U;
        }
        bridge_tx_queued++;
        if (bridge_tx_queued > raw_uart_bridge_queue_high_water)
        {
            raw_uart_bridge_queue_high_water = bridge_tx_queued;
        }
    }
}

static void RawUartBridge_StartTransmit(void)
{
    uint16_t contiguous_length;

    if (bridge_tx_busy || (bridge_tx_queued < RAW_BRIDGE_PACKET_SIZE))
    {
        return;
    }

    /* The sender always supplies 8-byte packets. Only preserve that boundary;
     * the packet content is never inspected or modified. */
    contiguous_length = RAW_BRIDGE_PACKET_SIZE;

    bridge_tx_transfer_length = contiguous_length;
    bridge_tx_complete = false;
    bridge_tx_busy = true;
    if (HAL_UART_Transmit_DMA(bridge_tx_uart,
                              &bridge_tx_queue[bridge_tx_read_pos],
                              contiguous_length) != HAL_OK)
    {
        bridge_tx_transfer_length = 0U;
        bridge_tx_busy = false;
        raw_uart_bridge_tx_error_count++;
    }
}

HAL_StatusTypeDef RawUartBridge_Init(UART_HandleTypeDef *rx_uart,
                                     UART_HandleTypeDef *tx_uart)
{
    if ((rx_uart == NULL) || (tx_uart == NULL) ||
        (rx_uart->Instance != USART6) || (tx_uart->Instance != UART7) ||
        (rx_uart->hdmarx == NULL) || (tx_uart->hdmatx == NULL) ||
        (rx_uart->hdmarx->Init.Mode != DMA_CIRCULAR) ||
        ((rx_uart->Init.Mode & UART_MODE_RX) == 0U) ||
        ((tx_uart->Init.Mode & UART_MODE_TX) == 0U) ||
        (tx_uart->Init.BaudRate != 115200U) ||
        (tx_uart->Init.WordLength != UART_WORDLENGTH_8B) ||
        (tx_uart->Init.StopBits != UART_STOPBITS_1) ||
        (tx_uart->Init.Parity != UART_PARITY_NONE))
    {
        return HAL_ERROR;
    }
    if (bridge_initialized)
    {
        return ((bridge_rx_uart == rx_uart) && (bridge_tx_uart == tx_uart)) ?
               HAL_OK : HAL_ERROR;
    }

    bridge_rx_uart = rx_uart;
    bridge_tx_uart = tx_uart;
    (void)memset(bridge_rx_dma_buffer, 0, sizeof(bridge_rx_dma_buffer));
    bridge_rx_read_pos = 0U;
    bridge_tx_read_pos = 0U;
    bridge_tx_write_pos = 0U;
    bridge_tx_queued = 0U;
    bridge_tx_transfer_length = 0U;
    bridge_tx_complete = false;
    bridge_rx_restart_requested = false;
    bridge_tx_restart_requested = false;
    bridge_tx_busy = false;
    raw_uart_bridge_rx_bytes = 0U;
    raw_uart_bridge_tx_bytes = 0U;
    raw_uart_bridge_dropped_bytes = 0U;
    raw_uart_bridge_rx_error_count = 0U;
    raw_uart_bridge_tx_error_count = 0U;
    raw_uart_bridge_queue_high_water = 0U;

    if (RawUartBridge_StartReceive() != HAL_OK)
    {
        bridge_rx_uart = NULL;
        bridge_tx_uart = NULL;
        return HAL_ERROR;
    }

    bridge_initialized = true;
    return HAL_OK;
}

void RawUartBridge_Run(void)
{
    if (!bridge_initialized)
    {
        return;
    }

    if (bridge_rx_restart_requested)
    {
        bridge_rx_restart_requested = false;
        (void)HAL_UART_AbortReceive(bridge_rx_uart);
        if (RawUartBridge_StartReceive() != HAL_OK)
        {
            bridge_rx_restart_requested = true;
        }
    }

    if (bridge_tx_restart_requested)
    {
        bridge_tx_restart_requested = false;
        (void)HAL_UART_AbortTransmit(bridge_tx_uart);
        bridge_tx_complete = false;
        bridge_tx_transfer_length = 0U;
        bridge_tx_busy = false;
    }

    RawUartBridge_FinishTransmit();
    RawUartBridge_QueueReceived();
    RawUartBridge_StartTransmit();
}

void RawUartBridge_HandleTxCplt(UART_HandleTypeDef *uart)
{
    if (bridge_initialized && (uart == bridge_tx_uart))
    {
        bridge_tx_complete = true;
    }
}

void RawUartBridge_HandleUartError(UART_HandleTypeDef *uart)
{
    if (!bridge_initialized)
    {
        return;
    }

    if (uart == bridge_rx_uart)
    {
        raw_uart_bridge_rx_error_count++;
        bridge_rx_restart_requested = true;
    }
    else if (uart == bridge_tx_uart)
    {
        raw_uart_bridge_tx_error_count++;
        bridge_tx_restart_requested = true;
    }
}
