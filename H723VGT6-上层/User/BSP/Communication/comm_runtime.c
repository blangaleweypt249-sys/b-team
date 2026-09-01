/**
 * @file comm_runtime.c
 * @brief 实现 UART、SPI、FDCAN 通信运行时及 DMA 缓存维护。
 */

#include "comm_runtime.h"

#include <string.h>

#include "bsp_can.h"
#include "fdcan.h"
#include "fdcan_dlc.h"
#include "usart.h"

/** 每个 UART DMA 循环接收缓冲区的字节数。 */
#define UART_DMA_RX_BUFFER_SIZE 256U
/** 通信运行时管理的 UART 通道总数。 */
#define UART_CHANNEL_COUNT      ((uint32_t)COMM_UART_CHANNEL_COUNT)
/** 可用于普通串口通信的非 RS485 通道数量。 */
#define UART_NON_RS485_COUNT    ((uint32_t)COMM_UART_RS485_1)
/** 向辅助控制板转发控制帧所用 UART 的通道索引。 */
#define UART_AUX_BRIDGE_INDEX   ((uint32_t)COMM_UART_UART5)
/** 通信运行时管理的 FDCAN 控制器数量。 */
#define FDCAN_CHANNEL_COUNT     3U
/** STM32H7 数据缓存行的字节数，用于 DMA 缓存对齐。 */
#define DCACHE_LINE_SIZE        32U

/** 保存 模块 运行过程中需要集中管理的数据。 */
typedef struct
{
  UART_HandleTypeDef *handle; /**< 该通道绑定的 HAL UART 外设句柄。 */
  uint8_t *buffer; /**< 用于暂存尚未完成解析的数据字节。 */
  uint32_t event_flag; /**< 该通道收到数据时通知通信任务的事件位。 */
  bool use_dma; /**< 该 UART 通道是否采用 DMA 循环接收。 */
  volatile uint32_t produced; /**< DMA 或中断已经写入接收缓冲区的累计字节位置。 */
  volatile uint16_t isr_position; /**< UART 中断接收写入 DMA 缓冲区的当前位置。 */
  volatile uint8_t restart_requested; /**< UART 出错后是否等待通信任务重新启动接收。 */
  uint32_t consumed; /**< 通信任务已经处理的累计字节位置。 */
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
  { &huart5,   uart5_rx_buffer,   COMM_EVENT_UART5_RX,   true,  0U, 0U, 0U, 0U },
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
/* 复制异步回复，避免调用方的栈缓冲区失效后影响发送。 */
__ALIGNED(DCACHE_LINE_SIZE) static uint8_t comm_pc_async_tx_buffer[256U];

volatile uint32_t comm_uart_rx_bytes[UART_CHANNEL_COUNT];
volatile uint32_t comm_uart_rx_isr_bytes[UART_CHANNEL_COUNT];
volatile uint32_t comm_uart_rx_event_count[UART_CHANNEL_COUNT];
volatile uint32_t comm_uart_overrun_count[UART_CHANNEL_COUNT];
volatile uint32_t comm_uart_error_count[UART_CHANNEL_COUNT];
volatile uint32_t comm_uart_restart_count[UART_CHANNEL_COUNT];
volatile uint32_t comm_uart_started_mask;
volatile uint32_t comm_fdcan_rx_count[FDCAN_CHANNEL_COUNT];
volatile uint32_t comm_notify_error_count;

/* 功能：计算覆盖缓冲区的 D-Cache 对齐范围；用途：为 DMA 缓存维护提供合法地址和长度；结果写入 start 与 length。 */
static void GetCacheRange(const void *buffer /* 函数读取或写入的对象地址 */,
                          size_t size /* 待处理数据的字节数 */,
                          uintptr_t *start /* 函数读取或写入的对象地址 */,
                          int32_t *length /* 函数读取或写入的对象地址 */)
{
  uintptr_t first;
  uintptr_t last;

  first = ((uintptr_t)buffer) & ~((uintptr_t)DCACHE_LINE_SIZE - 1U);
  last = (((uintptr_t)buffer) + size + DCACHE_LINE_SIZE - 1U) &
         ~((uintptr_t)DCACHE_LINE_SIZE - 1U);
  *start = first;
  *length = (int32_t)(last - first);
}

/* 功能：发送 DMA 前清理对应 D-Cache；用途：确保外设读到内存中的最新数据；无返回值表示缓存维护已完成。 */
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

/* 功能：接收 DMA 前清理并失效对应 D-Cache；用途：避免脏缓存覆盖外设写入；无返回值表示缓冲区已准备。 */
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

/* 功能：接收 DMA 后失效对应 D-Cache；用途：让 CPU 读取外设写入的最新数据；无返回值表示缓存已同步。 */
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

/* 功能：按 HAL UART 句柄查找运行时通道；用途：把中断回调映射到通道状态；返回 NULL 表示句柄未注册。 */
static UartDmaChannel *FindUartChannel(UART_HandleTypeDef *huart /* 函数读取或写入的对象地址 */,
                                       uint32_t *channel_index /* 函数读取或写入的对象地址 */)
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

/* 功能：复位通道计数并启动 UART 空闲接收；用途：初始化或错误后重启 DMA/中断接收；返回值表示 HAL 启动状态。 */
static HAL_StatusTypeDef StartUartReception(UartDmaChannel *channel /* 需要选择或上报的 UART 通道 */)
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

/* 功能：配置 FDCAN 标准帧接收过滤器；用途：接收全部标准数据帧并拒绝其他帧；返回值表示 HAL 配置状态。 */
static HAL_StatusTypeDef ConfigureFdcanFilter(FDCAN_HandleTypeDef *hfdcan /* 函数读取或写入的对象地址 */)
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

/* 功能：配置、启动 FDCAN 并打开接收通知；用途：使指定 CAN 外设进入工作状态；返回值表示启动是否成功。 */
static HAL_StatusTypeDef StartFdcan(FDCAN_HandleTypeDef *hfdcan /* 函数读取或写入的对象地址 */)
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

/* 功能：初始化所有 UART、FDCAN 和通知任务；用途：建立板级通信运行环境；返回 HAL_OK 表示全部通道启动成功。 */
HAL_StatusTypeDef CommRuntime_Init(osThreadId_t notify_task)
{
  uint32_t index;

  if (notify_task == NULL)
  {
    return HAL_ERROR;
  }

  comm_notify_task = notify_task;
  comm_uart_started_mask = 0U;

  for (index = 0U; index < UART_CHANNEL_COUNT; index++)
  {
    if (StartUartReception(&uart_channels[index]) != HAL_OK)
    {
      return HAL_ERROR;
    }
    comm_uart_started_mask |= 1UL << index;
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

/* 功能：消费指定 UART 通道的新数据并按需重启接收；用途：在任务上下文处理 DMA 环形缓冲区；无返回值表示数据已交给回调。 */
static void ProcessUartChannel(uint32_t index /* 需要处理的通道或数组下标 */)
{
  UartDmaChannel *channel;
  uint32_t produced;
  uint32_t pending;

  channel = &uart_channels[index];
  /* DMA 通道使用环形缓冲区；仅中断通道包含一个由空闲线界定的完整数据包，
     数据包处理完毕后重新启动接收。 */
  if (channel->use_dma && (channel->restart_requested != 0U))
  {
    comm_uart_restart_count[index]++;
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
    comm_uart_restart_count[index]++;
    (void)HAL_UART_AbortReceive(channel->handle);
    if (StartUartReception(channel) != HAL_OK)
    {
      comm_uart_error_count[index]++;
    }
  }
}

/* 功能：读空指定 FDCAN 接收 FIFO 并转换为通用帧；用途：在任务上下文分发 CAN 数据；无返回值表示当前积压已处理。 */
static void ProcessFdcanChannel(uint32_t index /* 需要处理的通道或数组下标 */)
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
      frame.dlc = FdcanDlc_DecodeClassic(header.DataLength);
      (void)memcpy(frame.data, data, sizeof(frame.data));
      comm_can_handler((uint8_t)(index + 1U),
                       &frame,
                       comm_handler_user_data);
    }
  }
}

/* 功能：按线程事件标志处理 UART 与 FDCAN 通道；用途：作为通信任务的统一调度入口；无返回值表示对应事件已消费。 */
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

/* 功能：在指定 UART 通道发起异步发送；用途：统一 DMA 与中断发送方式；返回值表示 HAL 是否接受发送请求。 */
static HAL_StatusTypeDef CommRuntime_UartTransmitAsync(UartDmaChannel *channel /* 需要选择或上报的 UART 通道 */,
                                                       const uint8_t *data /* 待处理数据的首地址 */,
                                                       uint16_t size /* 待处理数据的字节数 */)
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

/* 功能：通过 UART5 发送辅助控制帧；用途：把辅助输出状态送到抬升 H723；返回 true 表示发送已启动。 */
bool CommRuntime_AuxUartTransmit(const uint8_t *data, uint16_t size)
{
  if ((data == NULL) || (size == 0U) ||
      (huart5.gState != HAL_UART_STATE_READY))
  {
    return false;
  }
  return HAL_UART_Transmit_IT(&huart5, (uint8_t *)data, size) == HAL_OK;
}

/* 功能：检查 UART5 辅助发送通道是否空闲；用途：避免覆盖正在发送的帧。 */
bool CommRuntime_AuxUartTxReady(void)
{
  return huart5.gState == HAL_UART_STATE_READY;
}

/* 功能：取得当前上位机控制 UART 通道；用途：统一控制链路的收发选择并提供默认回退；返回值为有效通道指针。 */
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

/* 功能：注册 UART、CAN 数据处理器及用户上下文；用途：把板级通信接入应用层；无返回值表示后续数据将调用新处理器。 */
void CommRuntime_SetHandlers(comm_uart_handler_t uart_handler,
                             comm_can_handler_t can_handler,
                             void *user_data)
{
  comm_uart_handler = uart_handler;
  comm_can_handler = can_handler;
  comm_handler_user_data = user_data;
}

/* 功能：查询上位机 UART 是否可发送；用途：避免覆盖正在进行的异步发送；返回 true 表示通道空闲。 */
bool CommRuntime_PcTxReady(void)
{
  return GetPcUartChannel()->handle->gState == HAL_UART_STATE_READY;
}

/* 功能：通过当前控制 UART 异步发送数据；用途：发送协议响应和状态；返回 true 表示发送请求已被 HAL 接受。 */
bool CommRuntime_PcTransmit(const uint8_t *data, uint16_t size)
{
  return CommRuntime_UartTransmitAsync(GetPcUartChannel(), data, size) == HAL_OK;
}

/* 功能：复制并异步发送启动或诊断消息；用途：支持栈上数据且不阻塞任务。 */
bool CommRuntime_PcTransmitCopy(const uint8_t *data, uint16_t size)
{
  UartDmaChannel *channel;

  if ((data == NULL) || (size == 0U) ||
      (size > sizeof(comm_pc_async_tx_buffer)))
  {
    return false;
  }

  channel = GetPcUartChannel();
  if (channel->handle->gState != HAL_UART_STATE_READY)
  {
    return false;
  }

  (void)memcpy(comm_pc_async_tx_buffer, data, size);
  DmaCache_PrepareTx(comm_pc_async_tx_buffer, size);
  return HAL_UART_Transmit_DMA(channel->handle,
                               comm_pc_async_tx_buffer,
                               size) == HAL_OK;
}

/* 功能：设置上位机控制所用 UART；用途：选择协议收发通道；无返回值表示仅接受合法的 PC 通道枚举。 */
void CommRuntime_SetPcChannel(comm_uart_channel_t channel)
{
  if (channel == COMM_UART_PC)
  {
    comm_pc_channel = COMM_UART_PC;
  }
}

/* 功能：查询当前上位机控制 UART；用途：供数据分发和诊断判断通道；返回值表示通道枚举。 */
comm_uart_channel_t CommRuntime_GetPcChannel(void)
{
  return comm_pc_channel;
}

/* 功能：检查除控制口外的遥测 UART 是否全部空闲；用途：保证广播发送可同时启动；返回 true 表示可发送。 */
bool CommRuntime_TelemetryTxReady(void)
{
  uint32_t index;
  uint32_t control_index;

  control_index = (uint32_t)comm_pc_channel;
  for (index = 0U; index < UART_NON_RS485_COUNT; index++)
  {
    if ((index == control_index) || (index == UART_AUX_BRIDGE_INDEX))
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

/* 功能：向所有非控制、非 RS485 UART 广播遥测；用途：输出 VOFA 等监测数据；返回 true 表示所有发送请求均启动成功。 */
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
    if ((index == control_index) || (index == UART_AUX_BRIDGE_INDEX))
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
    if ((index == control_index) || (index == UART_AUX_BRIDGE_INDEX))
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

/* 功能：取得系统毫秒计数；用途：为通信和控制模块提供统一时基；返回值等同 HAL_GetTick。 */
uint32_t CommRuntime_GetTickMs(void)
{
  return HAL_GetTick();
}

/* 功能：向通信任务设置事件标志；用途：从 ISR 唤醒对应通道处理；失败时递增通知错误计数。 */
static void NotifyCommTask(uint32_t flag /* 需要通知通信任务的单个事件位 */)
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

/* 功能：处理 HAL UART 空闲接收事件；用途：累计新字节位置并通知通信任务；这是由 HAL 在中断上下文调用的回调。 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  UartDmaChannel *channel;
  uint32_t index;
  uint16_t position;
  uint16_t previous;
  uint16_t delta;

  channel = FindUartChannel(huart, &index);
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
  comm_uart_rx_event_count[index]++;
  comm_uart_rx_isr_bytes[index] += delta;
  if (!channel->use_dma)
  {
    channel->restart_requested = 1U;
  }
  NotifyCommTask(channel->event_flag);
}

/* 功能：处理 HAL UART 错误事件；用途：记录错误、请求重启接收并唤醒通信任务；这是中断上下文回调。 */
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

/* 功能：处理 FDCAN FIFO0 新消息通知；用途：将对应总线事件交给通信任务；这是由 HAL 在中断上下文调用的回调。 */
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
