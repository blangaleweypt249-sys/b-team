#ifndef DM_2006_BUS_H
#define DM_2006_BUS_H

#include "fdcan.h"

#include <stdint.h>

#define DM_2006_BUS_RX_QUEUE_SIZE 32U
#define DM_2006_BUS_MAX_HANDLERS  2U

typedef void (*dm_2006_bus_rx_handler_t)(void *context, uint16_t id,
                                         const uint8_t data[8],
                                         uint32_t tick_ms);

typedef struct
{
    uint16_t id;
    uint32_t tick_ms;
    uint8_t data[8];
} dm_2006_bus_frame_t;

typedef struct
{
    dm_2006_bus_rx_handler_t callback;
    void *context;
} dm_2006_bus_handler_t;

typedef struct
{
    uint32_t rx_count;
    uint32_t rx_overflow_count;
    uint32_t rx_error_count;
    uint32_t tx_count;
    uint32_t tx_busy_count;
    uint32_t tx_error_count;
    uint32_t error_warning_count;
    uint32_t error_passive_count;
    uint32_t bus_off_count;
} dm_2006_bus_diag_t;

typedef struct
{
    FDCAN_HandleTypeDef *device;
    dm_2006_bus_handler_t handlers[DM_2006_BUS_MAX_HANDLERS];
    dm_2006_bus_frame_t rx_queue[DM_2006_BUS_RX_QUEUE_SIZE];
    volatile uint8_t rx_head;
    volatile uint8_t rx_tail;
    uint8_t handler_count;
    uint8_t ready;
    dm_2006_bus_diag_t diag;
} dm_2006_bus_t;

HAL_StatusTypeDef DM2006_BusInit(dm_2006_bus_t *bus,
                                 FDCAN_HandleTypeDef *device,
                                 const uint16_t *rx_ids,
                                 uint8_t rx_id_count);
void DM2006_BusStop(dm_2006_bus_t *bus);
HAL_StatusTypeDef DM2006_BusAddRxHandler(
    dm_2006_bus_t *bus, dm_2006_bus_rx_handler_t callback, void *context);
HAL_StatusTypeDef DM2006_BusSend(dm_2006_bus_t *bus, uint16_t id,
                                 const uint8_t data[8]);
void DM2006_BusHandleRxInterrupt(dm_2006_bus_t *bus,
                                 uint32_t interrupt_flags);
void DM2006_BusHandleErrorInterrupt(dm_2006_bus_t *bus,
                                    uint32_t interrupt_flags);
void DM2006_BusProcessRx(dm_2006_bus_t *bus);
void DM2006_BusGetDiag(const dm_2006_bus_t *bus,
                       dm_2006_bus_diag_t *diag);

#endif
