#ifndef RS_BUS_H
#define RS_BUS_H

#include "fdcan.h"

#include <stdint.h>

#define RS_BUS_RX_QUEUE_SIZE 32U

typedef void (*rs_bus_rx_handler_t)(void *context, uint32_t id,
                                    const uint8_t data[8], uint32_t tick_ms);

typedef struct
{
    uint32_t id;
    uint32_t tick_ms;
    uint8_t data[8];
} rs_bus_frame_t;

typedef struct
{
    FDCAN_HandleTypeDef *device;
    rs_bus_rx_handler_t rx_handler;
    void *rx_context;
    rs_bus_frame_t rx_queue[RS_BUS_RX_QUEUE_SIZE];
    volatile uint8_t rx_head;
    volatile uint8_t rx_tail;
    uint8_t host_id;
    uint8_t ready;
} rs_bus_t;

HAL_StatusTypeDef RS_BusInit(rs_bus_t *bus, FDCAN_HandleTypeDef *device,
                             uint8_t host_id);
void RS_BusStop(rs_bus_t *bus);
HAL_StatusTypeDef RS_BusSetRxHandler(rs_bus_t *bus,
                                     rs_bus_rx_handler_t handler,
                                     void *context);
HAL_StatusTypeDef RS_BusSend(rs_bus_t *bus, uint32_t id,
                             const uint8_t data[8]);
void RS_BusHandleRxInterrupt(rs_bus_t *bus, uint32_t interrupt_flags);
void RS_BusProcessRx(rs_bus_t *bus);

#endif
