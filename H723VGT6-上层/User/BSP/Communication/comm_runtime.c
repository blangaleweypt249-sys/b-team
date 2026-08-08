#include "comm_runtime.h"

#include <string.h>

#include "can_id.h"
#include "fdcan.h"
#include "usart.h"

#define UART_DMA_RX_BUFFER_SIZE 256U
#define UART_CHANNEL_COUNT      ((uint32_t)COMM_UART_CHANNEL_COUNT)
#define UART_NON_RS485_COUNT    ((uint32_t)COMM_UART_RS485_1)
#define FDCAN_CHANNEL_COUNT     3U
#define DCACHE_LINE_SIZE        32U

typedef struct
{
  UART_HandleTypeDef *handle;
  uint8_t *buffer;
  uint32_t event_flag;
  bool use_dma;
  volatile uint32_t produced;
  volatile uint16_t isr_position;
  volatile uint8_t restart_requested;
  uint32_t consumed;
} UartDmaChannel;

__ALIGNED(DCACHE_LINE_SIZE) static uint8_t uart4_rx_buffer[UART_DMA_RX_BUFFER_SIZE];
__ALIGNED(DCACHE_LINE_SIZE) static uint8_t uart5_rx_buffer[UART_DMA_RX_BUFFER_SIZE];
__ALIGNED(DCACHE_LINE_SIZE) static uint8_t uart7_rx_buffer[UART_DMA_RX_BUFFER_SIZE];
__ALIGNED(DCACHE_LINE_SIZE) static uint8_t uart8_rx_buffer[UART_DMA_RX_BUFFER_SIZE];
__ALIGNED(DCACHE_LINE_SIZE) static uint8_t uart9_rx_buffer[UART_DMA_RX_BUFFER_SIZE];
__ALIGNED(DCACHE_LINE_SIZE) static uint8_t usart3_rx_buffer[UART_DMA_RX_BUFFER_SIZE];
__ALIGNED(DCACHE_LINE_SIZE) static uint8_t usart6_rx_buffer[UART_DMA_RX_BUFFER_SIZE];
__ALIGNED(DCACHE_LINE_SIZE) static uint8_t usart10_rx_buffer[UART_DMA_RX_BUFFER_SIZE];
__ALIGNED(DCACHE_LINE_SIZE) static uint8_t usart1_rx_buffer[UART_DMA_RX_BUFFER_SIZE];
__ALIGNED(DCACHE_LINE_SIZE) static uint8_t usart2_rx_buffer[UART_DMA_RX_BUFFER_SIZE];

static UartDmaChannel uart_channels[UART_CHANNEL_COUNT] =
{
  { &huart4,   uart4_rx_buffer,   COMM_EVENT_UART4_RX,   true,  0U, 0U, 0U, 0U },
  { &huart5,   uart5_rx_buffer,   COMM_EVENT_UART5_RX,   false, 0U, 0U, 0U, 0U },
  { &huart7,   uart7_rx_buffer,   COMM_EVENT_UART7_RX,   false, 0U, 0U, 0U, 0U },
  { &huart8,   uart8_rx_buffer,   COMM_EVENT_UART8_RX,   false, 0U, 0U, 0U, 0U },
  { &huart9,   uart9_rx_buffer,   COMM_EVENT_UART9_RX,   false, 0U, 0U, 0U, 0U },
  { &huart3,   usart3_rx_buffer,  COMM_EVENT_USART3_RX,  false, 0U, 0U, 0U, 0U },
  { &huart6,   usart6_rx_buffer,  COMM_EVENT_USART6_RX,  false, 0U, 0U, 0U, 0U },
  { &huart10,  usart10_rx_buffer, COMM_EVENT_USART10_RX, false, 0U, 0U, 0U, 0U },
  { &huart1,   usart1_rx_buffer,  COMM_EVENT_USART1_RX,  true,  0U, 0U, 0U, 0U },
  { &huart2,   usart2_rx_buffer,  COMM_EVENT_USART2_RX,  true,  0U, 0U, 0U, 0U }
};

static FDCAN_HandleTypeDef *const fdcan_channels[FDCAN_CHANNEL_COUNT] =
{
  &hfdcan1,
  &hfdcan2,
  &hfdcan3
};

static const uint32_t fdcan_event_flags[FDCAN_CHANNEL_COUNT] =
{
  COMM_EVENT_FDCAN1_RX,
  COMM_EVENT_FDCAN2_RX,
  COMM_EVENT_FDCAN3_RX
};

static osThreadId_t comm_notify_task;
static comm_uart_handler_t comm_uart_handler;
static comm_can_handler_t comm_can_handler;
static void *comm_handler_user_data;
static volatile comm_uart_channel_t comm_pc_channel = COMM_UART_UART4;

volatile uint32_t comm_uart_rx_bytes[UART_CHANNEL_COUNT];
volatile uint32_t comm_uart_overrun_count[UART_CHANNEL_COUNT];
volatile uint32_t comm_uart_error_count[UART_CHANNEL_COUNT];
volatile uint32_t comm_fdcan_rx_count[FDCAN_CHANNEL_COUNT];
volatile uint32_t comm_notify_error_count;

static void GetCacheRange(const void *buffer,
                          size_t size,
                          uintptr_t *start,
                          int32_t *length)
{
  uintptr_t first;
  uintptr_t last;

  first = ((uintptr_t)buffer) & ~((uintptr_t)DCACHE_LINE_SIZE - 1U);
  last = (((uintptr_t)buffer) + size + DCACHE_LINE_SIZE - 1U) &
         ~((uintptr_t)DCACHE_LINE_SIZE - 1U);
  *start = first;
  *length = (int32_t)(last - first);
}

void DmaCache_PrepareTx(const void *buffer, size_t size)
{
  uintptr_t start;
  int32_t length;

  if ((buffer == NULL) || (size == 0U))
  {
    return;
  }

  GetCacheRange(buffer, size, &start, &length);
  SCB_CleanDCache_by_Addr((void *)start, length);
  __DSB();
}

void DmaCache_PrepareRx(void *buffer, size_t size)
{
  uintptr_t start;
  int32_t length;

  if ((buffer == NULL) || (size == 0U))
  {
    return;
  }

  GetCacheRange(buffer, size, &start, &length);
  SCB_CleanInvalidateDCache_by_Addr((void *)start, length);
  __DSB();
}

void DmaCache_CompleteRx(void *buffer, size_t size)
{
  uintptr_t start;
  int32_t length;

  if ((buffer == NULL) || (size == 0U))
  {
    return;
  }

  GetCacheRange(buffer, size, &start, &length);
  SCB_InvalidateDCache_by_Addr((void *)start, length);
  __DSB();
}

static UartDmaChannel *FindUartChannel(UART_HandleTypeDef *huart,
                                       uint32_t *channel_index)
{
  uint32_t index;

  for (index = 0U; index < UART_CHANNEL_COUNT; index++)
  {
    if (uart_channels[index].handle == huart)
    {
      if (channel_index != NULL)
      {
        *channel_index = index;
      }
      return &uart_channels[index];
    }
  }

  return NULL;
}

static HAL_StatusTypeDef StartUartReception(UartDmaChannel *channel)
{
  HAL_StatusTypeDef status;

  channel->produced = 0U;
  channel->consumed = 0U;
  channel->isr_position = 0U;
  channel->restart_requested = 0U;
  DmaCache_PrepareRx(channel->buffer, UART_DMA_RX_BUFFER_SIZE);

  if (channel->use_dma)
  {
    status = HAL_UARTEx_ReceiveToIdle_DMA(channel->handle,
                                          channel->buffer,
                                          UART_DMA_RX_BUFFER_SIZE);
    if (status == HAL_OK)
    {
      __HAL_DMA_DISABLE_IT(channel->handle->hdmarx, DMA_IT_HT);
    }
  }
  else
  {
    status = HAL_UARTEx_ReceiveToIdle_IT(channel->handle,
                                         channel->buffer,
                                         UART_DMA_RX_BUFFER_SIZE);
  }

  return status;
}

static HAL_StatusTypeDef ConfigureFdcanFilter(FDCAN_HandleTypeDef *hfdcan)
{
  FDCAN_FilterTypeDef filter = {0};

  if ((hfdcan != &hfdcan1) && (hfdcan != &hfdcan2) &&
      (hfdcan != &hfdcan3))
  {
    return HAL_ERROR;
  }
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0U;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterType = FDCAN_FILTER_RANGE;
  filter.FilterID1 = 0x000U;
  filter.FilterID2 = 0x7FFU;

  if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                      FDCAN_REJECT,
                                      FDCAN_REJECT,
                                      FDCAN_REJECT_REMOTE,
                                      FDCAN_REJECT_REMOTE);
}

static HAL_StatusTypeDef StartFdcan(FDCAN_HandleTypeDef *hfdcan)
{
  if (ConfigureFdcanFilter(hfdcan) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_FDCAN_ActivateNotification(hfdcan,
                                         FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                         0U);
}

HAL_StatusTypeDef CommRuntime_Init(osThreadId_t notify_task)
{
  uint32_t index;

  if (notify_task == NULL)
  {
    return HAL_ERROR;
  }

  comm_notify_task = notify_task;

  for (index = 0U; index < UART_CHANNEL_COUNT; index++)
  {
    if (StartUartReception(&uart_channels[index]) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  for (index = 0U; index < FDCAN_CHANNEL_COUNT; index++)
  {
    if (StartFdcan(fdcan_channels[index]) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}

static void ProcessUartChannel(uint32_t index)
{
  UartDmaChannel *channel;
  uint32_t produced;
  uint32_t pending;

  channel = &uart_channels[index];
  /* DMA channels keep a circular buffer; interrupt-only channels contain one
     complete idle-delimited packet and are rearmed after it is consumed. */
  if (channel->use_dma && (channel->restart_requested != 0U))
  {
    (void)HAL_UART_AbortReceive(channel->handle);
    if (StartUartReception(channel) != HAL_OK)
    {
      comm_uart_error_count[index]++;
    }
    return;
  }

  if (channel->use_dma)
  {
    DmaCache_CompleteRx(channel->buffer, UART_DMA_RX_BUFFER_SIZE);
  }
  produced = channel->produced;
  pending = produced - channel->consumed;

  if (pending > UART_DMA_RX_BUFFER_SIZE)
  {
    comm_uart_overrun_count[index]++;
    channel->consumed = produced - UART_DMA_RX_BUFFER_SIZE;
    pending = UART_DMA_RX_BUFFER_SIZE;
  }

  while (pending > 0U)
  {
    uint32_t offset;
    uint32_t chunk;

    offset = channel->consumed % UART_DMA_RX_BUFFER_SIZE;
    chunk = UART_DMA_RX_BUFFER_SIZE - offset;
    if (chunk > pending)
    {
      chunk = pending;
    }

    if (comm_uart_handler != NULL)
    {
      comm_uart_handler((comm_uart_channel_t)index,
                        &channel->buffer[offset],
                        chunk,
                        comm_handler_user_data);
    }
    channel->consumed += chunk;
    comm_uart_rx_bytes[index] += chunk;
    pending -= chunk;
  }

  if (!channel->use_dma && (channel->restart_requested != 0U))
  {
    (void)HAL_UART_AbortReceive(channel->handle);
    if (StartUartReception(channel) != HAL_OK)
    {
      comm_uart_error_count[index]++;
    }
  }
}

static void ProcessFdcanChannel(uint32_t index)
{
  FDCAN_RxHeaderTypeDef header;
  can_frame_t frame;
  uint8_t data[8];
  FDCAN_HandleTypeDef *hfdcan;

  hfdcan = fdcan_channels[index];
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
  {
    (void)memset(data, 0, sizeof(data));
    if (HAL_FDCAN_GetRxMessage(hfdcan,
                               FDCAN_RX_FIFO0,
                               &header,
                               data) != HAL_OK)
    {
      break;
    }

    comm_fdcan_rx_count[index]++;
    if (comm_can_handler != NULL)
    {
      frame.id = header.Identifier;
      frame.extended = header.IdType == FDCAN_EXTENDED_ID;
      frame.dlc = (uint8_t)((header.DataLength >> 16U) & 0x0FU);
      if (frame.dlc > 8U)
      {
        frame.dlc = 8U;
      }
      (void)memcpy(frame.data, data, sizeof(frame.data));
      comm_can_handler((uint8_t)(index + 1U),
                       &frame,
                       comm_handler_user_data);
    }
  }
}

void CommRuntime_Process(uint32_t flags)
{
  uint32_t index;

  for (index = 0U; index < UART_CHANNEL_COUNT; index++)
  {
    if ((flags & uart_channels[index].event_flag) != 0U)
    {
      ProcessUartChannel(index);
    }
  }

  for (index = 0U; index < FDCAN_CHANNEL_COUNT; index++)
  {
    if ((flags & fdcan_event_flags[index]) != 0U)
    {
      ProcessFdcanChannel(index);
    }
  }
}

static HAL_StatusTypeDef CommRuntime_UartTransmitAsync(UartDmaChannel *channel,
                                                       const uint8_t *data,
                                                       uint16_t size)
{
  if ((channel == NULL) || (channel->handle == NULL) ||
      (data == NULL) || (size == 0U))
  {
    return HAL_ERROR;
  }

  if (channel->use_dma)
  {
    DmaCache_PrepareTx(data, size);
    return HAL_UART_Transmit_DMA(channel->handle, data, size);
  }
  return HAL_UART_Transmit_IT(channel->handle, data, size);
}

static UartDmaChannel *GetPcUartChannel(void)
{
  uint32_t index;

  index = (uint32_t)comm_pc_channel;
  if (index >= UART_NON_RS485_COUNT)
  {
    index = (uint32_t)COMM_UART_UART4;
  }
  return &uart_channels[index];
}

void CommRuntime_SetHandlers(comm_uart_handler_t uart_handler,
                             comm_can_handler_t can_handler,
                             void *user_data)
{
  comm_uart_handler = uart_handler;
  comm_can_handler = can_handler;
  comm_handler_user_data = user_data;
}

bool CommRuntime_PcTxReady(void)
{
  return GetPcUartChannel()->handle->gState == HAL_UART_STATE_READY;
}

bool CommRuntime_PcTransmit(const uint8_t *data, uint16_t size)
{
  return CommRuntime_UartTransmitAsync(GetPcUartChannel(), data, size) == HAL_OK;
}

bool CommRuntime_PcTransmitBlocking(const uint8_t *data,
                                    uint16_t size,
                                    uint32_t timeout_ms)
{
  UartDmaChannel *channel;
  uint32_t start_tick;

  if ((data == NULL) || (size == 0U) || (timeout_ms == 0U))
  {
    return false;
  }

  channel = GetPcUartChannel();
  start_tick = HAL_GetTick();
  while ((HAL_GetTick() - start_tick) < timeout_ms)
  {
    uint32_t elapsed;
    uint32_t remaining;

    if (channel->handle->gState != HAL_UART_STATE_READY)
    {
      (void)osDelay(1U);
      continue;
    }

    elapsed = HAL_GetTick() - start_tick;
    remaining = timeout_ms - elapsed;
    if (HAL_UART_Transmit(channel->handle,
                          (const uint8_t *)data,
                          size,
                          remaining) == HAL_OK)
    {
      return true;
    }
    (void)osDelay(1U);
  }

  return false;
}

void CommRuntime_SetPcChannel(comm_uart_channel_t channel)
{
  if ((uint32_t)channel < UART_NON_RS485_COUNT)
  {
    comm_pc_channel = channel;
  }
}

comm_uart_channel_t CommRuntime_GetPcChannel(void)
{
  return comm_pc_channel;
}

bool CommRuntime_TelemetryTxReady(void)
{
  uint32_t index;
  uint32_t control_index;

  control_index = (uint32_t)comm_pc_channel;
  for (index = 0U; index < UART_NON_RS485_COUNT; index++)
  {
    if (index == control_index)
    {
      continue;
    }
    if (uart_channels[index].handle->gState != HAL_UART_STATE_READY)
    {
      return false;
    }
  }
  return true;
}

bool CommRuntime_TelemetryTransmit(const uint8_t *data, uint16_t size)
{
  uint32_t index;
  uint32_t control_index;

  if ((data == NULL) || (size == 0U))
  {
    return false;
  }

  control_index = (uint32_t)comm_pc_channel;
  for (index = 0U; index < UART_NON_RS485_COUNT; index++)
  {
    if (index == control_index)
    {
      continue;
    }
    if (uart_channels[index].handle->gState != HAL_UART_STATE_READY)
    {
      return false;
    }
  }

  for (index = 0U; index < UART_NON_RS485_COUNT; index++)
  {
    if (index == control_index)
    {
      continue;
    }
    if (CommRuntime_UartTransmitAsync(&uart_channels[index], data, size) != HAL_OK)
    {
      return false;
    }
  }
  return true;
}

uint32_t CommRuntime_GetTickMs(void)
{
  return HAL_GetTick();
}

static void NotifyCommTask(uint32_t flag)
{
  uint32_t result;

  if (comm_notify_task == NULL)
  {
    comm_notify_error_count++;
    return;
  }

  result = osThreadFlagsSet(comm_notify_task, flag);
  if ((result & osFlagsError) != 0U)
  {
    comm_notify_error_count++;
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  UartDmaChannel *channel;
  uint16_t position;
  uint16_t previous;
  uint16_t delta;

  channel = FindUartChannel(huart, NULL);
  if (channel == NULL)
  {
    return;
  }

  previous = channel->isr_position;
  if (size >= UART_DMA_RX_BUFFER_SIZE)
  {
    position = 0U;
    delta = UART_DMA_RX_BUFFER_SIZE - previous;
  }
  else
  {
    position = size;
    delta = (position >= previous) ?
            (uint16_t)(position - previous) :
            (uint16_t)(UART_DMA_RX_BUFFER_SIZE - previous + position);
  }
  channel->isr_position = position;
  channel->produced += delta;
  if (!channel->use_dma)
  {
    channel->restart_requested = 1U;
  }
  NotifyCommTask(channel->event_flag);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  UartDmaChannel *channel;
  uint32_t index;

  channel = FindUartChannel(huart, &index);
  if (channel == NULL)
  {
    return;
  }

  comm_uart_error_count[index]++;
  channel->restart_requested = 1U;
  NotifyCommTask(channel->event_flag);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t rx_fifo0_its)
{
  uint32_t index;

  if ((rx_fifo0_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
  {
    return;
  }

  for (index = 0U; index < FDCAN_CHANNEL_COUNT; index++)
  {
    if (fdcan_channels[index] == hfdcan)
    {
      NotifyCommTask(fdcan_event_flags[index]);
      break;
    }
  }
}
