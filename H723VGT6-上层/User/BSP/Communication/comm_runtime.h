/**
 * @file comm_runtime.h
 * @brief 声明通信运行时、数据发送和 DMA 缓存维护接口。
 */

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

#define COMM_EVENT_UART4_RX    (1UL << 0)
#define COMM_EVENT_UART5_RX    (1UL << 1)
#define COMM_EVENT_UART7_RX    (1UL << 2)
#define COMM_EVENT_UART8_RX    (1UL << 3)
#define COMM_EVENT_UART9_RX    (1UL << 4)
#define COMM_EVENT_USART3_RX   (1UL << 5)
#define COMM_EVENT_USART6_RX   (1UL << 6)
#define COMM_EVENT_USART10_RX  (1UL << 7)
#define COMM_EVENT_USART1_RX   (1UL << 8)
#define COMM_EVENT_USART2_RX   (1UL << 9)
#define COMM_EVENT_FDCAN1_RX   (1UL << 10)
#define COMM_EVENT_FDCAN2_RX   (1UL << 11)
#define COMM_EVENT_FDCAN3_RX   (1UL << 12)
#define COMM_EVENT_ALL         (COMM_EVENT_UART4_RX | COMM_EVENT_UART5_RX | \
                                COMM_EVENT_UART7_RX | \
                                COMM_EVENT_UART8_RX | COMM_EVENT_UART9_RX | \
                                COMM_EVENT_USART3_RX | COMM_EVENT_USART6_RX | \
                                COMM_EVENT_USART10_RX | COMM_EVENT_USART1_RX | \
                                COMM_EVENT_USART2_RX | COMM_EVENT_FDCAN1_RX | \
                                COMM_EVENT_FDCAN2_RX | COMM_EVENT_FDCAN3_RX)

typedef enum
{
    /* COMM_UART_RS485_1 之前的所有通道均为普通 TTL UART。 */
    COMM_UART_UART4,
    COMM_UART_UART5,
    COMM_UART_UART7,
    COMM_UART_UART8,
    COMM_UART_UART9,
    COMM_UART_USART3,
    COMM_UART_USART6,
    COMM_UART_USART10,
    COMM_UART_RS485_1,
    COMM_UART_RS485_2,
    COMM_UART_CHANNEL_COUNT
} comm_uart_channel_t;

#define COMM_UART_PC COMM_UART_UART4

typedef void (*comm_uart_handler_t)(comm_uart_channel_t channel,
                                    const uint8_t *data,
                                    size_t size,
                                    void *user_data);
typedef void (*comm_can_handler_t)(uint8_t can_bus,
                                   const can_frame_t *frame,
                                   void *user_data);

/* 功能：初始化所有 UART、FDCAN 和通知任务；用途：建立板级通信运行环境；返回 HAL_OK 表示全部通道启动成功。 */
HAL_StatusTypeDef CommRuntime_Init(osThreadId_t notify_task);
/* 功能：按线程事件标志处理 UART 与 FDCAN 通道；用途：作为通信任务的统一调度入口；无返回值表示对应事件已消费。 */
void CommRuntime_Process(uint32_t flags);
/* 功能：注册 UART、CAN 数据处理器及用户上下文；用途：把板级通信接入应用层；无返回值表示后续数据将调用新处理器。 */
void CommRuntime_SetHandlers(comm_uart_handler_t uart_handler,
                             comm_can_handler_t can_handler,
                             void *user_data);
/* 功能：查询上位机 UART 是否可发送；用途：避免覆盖正在进行的异步发送；返回 true 表示通道空闲。 */
bool CommRuntime_PcTxReady(void);
/* 功能：通过当前控制 UART 异步发送数据；用途：发送协议响应和状态；返回 true 表示发送请求已被 HAL 接受。 */
bool CommRuntime_PcTransmit(const uint8_t *data, uint16_t size);
/* 功能：复制并异步发送启动或诊断消息；用途：支持栈上数据且不阻塞任务。 */
bool CommRuntime_PcTransmitCopy(const uint8_t *data, uint16_t size);
/* 功能：通过 UART5 发送辅助控制帧；用途：向抬升 H723 转发辅助输出状态。 */
bool CommRuntime_AuxUartTransmit(const uint8_t *data, uint16_t size);
/* 功能：检查 UART5 辅助发送通道是否空闲；用途：避免覆盖正在发送的帧。 */
bool CommRuntime_AuxUartTxReady(void);
/* 功能：设置上位机控制所用 UART；用途：选择协议收发通道；无返回值表示仅接受合法的 PC 通道枚举。 */
void CommRuntime_SetPcChannel(comm_uart_channel_t channel);
/* 功能：查询当前上位机控制 UART；用途：供数据分发和诊断判断通道；返回值表示通道枚举。 */
comm_uart_channel_t CommRuntime_GetPcChannel(void);
/* 功能：检查除控制口外的遥测 UART 是否全部空闲；用途：保证广播发送可同时启动；返回 true 表示可发送。 */
bool CommRuntime_TelemetryTxReady(void);
/* 功能：向所有非控制、非 RS485 UART 广播遥测；用途：输出 VOFA 等监测数据；返回 true 表示所有发送请求均启动成功。 */
bool CommRuntime_TelemetryTransmit(const uint8_t *data, uint16_t size);
/* 功能：取得系统毫秒计数；用途：为通信和控制模块提供统一时基；返回值等同 HAL_GetTick。 */
uint32_t CommRuntime_GetTickMs(void);

/* 功能：发送 DMA 前清理对应 D-Cache；用途：确保外设读到内存中的最新数据；无返回值表示缓存维护已完成。 */
void DmaCache_PrepareTx(const void *buffer, size_t size);
/* 功能：接收 DMA 前清理并失效对应 D-Cache；用途：避免脏缓存覆盖外设写入；无返回值表示缓冲区已准备。 */
void DmaCache_PrepareRx(void *buffer, size_t size);
/* 功能：接收 DMA 后失效对应 D-Cache；用途：让 CPU 读取外设写入的最新数据；无返回值表示缓存已同步。 */
void DmaCache_CompleteRx(void *buffer, size_t size);

extern volatile uint32_t comm_uart_rx_bytes[COMM_UART_CHANNEL_COUNT];
extern volatile uint32_t comm_uart_rx_isr_bytes[COMM_UART_CHANNEL_COUNT];
extern volatile uint32_t comm_uart_rx_event_count[COMM_UART_CHANNEL_COUNT];
extern volatile uint32_t comm_uart_overrun_count[COMM_UART_CHANNEL_COUNT];
extern volatile uint32_t comm_uart_error_count[COMM_UART_CHANNEL_COUNT];
extern volatile uint32_t comm_uart_restart_count[COMM_UART_CHANNEL_COUNT];
extern volatile uint32_t comm_uart_started_mask;
extern volatile uint32_t comm_fdcan_rx_count[3];
extern volatile uint32_t comm_notify_error_count;

#ifdef __cplusplus
}
#endif

#endif
