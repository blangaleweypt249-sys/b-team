#include "dm_2006_bus.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define DM_2006_BUS_RX_QUEUE_MASK (DM_2006_BUS_RX_QUEUE_SIZE - 1U)
#define DM_2006_BUS_STD_ID_MAX    0x7FFU

static HAL_StatusTypeDef configure_filters(FDCAN_HandleTypeDef *device,
                                            const uint16_t *rx_ids,
                                            uint8_t rx_id_count)
{
    uint8_t filter_count = (uint8_t)((rx_id_count + 1U) / 2U);
    uint8_t i;

    if (filter_count > device->Init.StdFiltersNbr)
    {
        return HAL_ERROR;
    }

    for (i = 0U; i < filter_count; i++)
    {
        uint8_t first = (uint8_t)(i * 2U);
        FDCAN_FilterTypeDef filter = {0};

        filter.IdType = FDCAN_STANDARD_ID;
        filter.FilterIndex = i;
        filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        filter.FilterID1 = rx_ids[first];
        if ((uint8_t)(first + 1U) < rx_id_count)
        {
            filter.FilterType = FDCAN_FILTER_DUAL;
            filter.FilterID2 = rx_ids[first + 1U];
        }
        else
        {
            filter.FilterType = FDCAN_FILTER_MASK;
            filter.FilterID2 = DM_2006_BUS_STD_ID_MAX;
        }

        if (HAL_FDCAN_ConfigFilter(device, &filter) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

static bool rx_ids_valid(const uint16_t *rx_ids, uint8_t rx_id_count)
{
    uint8_t i;
    uint8_t j;

    if ((rx_ids == NULL) || (rx_id_count == 0U))
    {
        return false;
    }

    for (i = 0U; i < rx_id_count; i++)
    {
        if (rx_ids[i] > DM_2006_BUS_STD_ID_MAX)
        {
            return false;
        }
        for (j = (uint8_t)(i + 1U); j < rx_id_count; j++)
        {
            if (rx_ids[i] == rx_ids[j])
            {
                return false;
            }
        }
    }

    return true;
}

HAL_StatusTypeDef DM2006_BusInit(dm_2006_bus_t *bus,
                                 FDCAN_HandleTypeDef *device,
                                 const uint16_t *rx_ids,
                                 uint8_t rx_id_count)
{
    HAL_StatusTypeDef status;

    if ((bus == NULL) || (device == NULL) ||
        !rx_ids_valid(rx_ids, rx_id_count))
    {
        return HAL_ERROR;
    }

    memset(bus, 0, sizeof(*bus));
    bus->device = device;

    status = configure_filters(device, rx_ids, rx_id_count);
    if (status == HAL_OK)
    {
        status = HAL_FDCAN_ConfigGlobalFilter(
            device, FDCAN_REJECT, FDCAN_REJECT,
            FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    }
    if (status == HAL_OK)
    {
        status = HAL_FDCAN_Start(device);
    }
    if (status == HAL_OK)
    {
        status = HAL_FDCAN_ActivateNotification(
            device,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_ERROR_WARNING |
                FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_BUS_OFF,
            0U);
    }
    if (status != HAL_OK)
    {
        (void)HAL_FDCAN_Stop(device);
        return status;
    }

    bus->ready = 1U;
    return HAL_OK;
}

void DM2006_BusStop(dm_2006_bus_t *bus)
{
    if ((bus == NULL) || (bus->ready == 0U))
    {
        return;
    }

    (void)HAL_FDCAN_Stop(bus->device);
    bus->ready = 0U;
}

HAL_StatusTypeDef DM2006_BusAddRxHandler(
    dm_2006_bus_t *bus, dm_2006_bus_rx_handler_t callback, void *context)
{
    uint8_t i;

    if ((bus == NULL) || (bus->ready == 0U) || (callback == NULL))
    {
        return HAL_ERROR;
    }

    for (i = 0U; i < bus->handler_count; i++)
    {
        if ((bus->handlers[i].callback == callback) &&
            (bus->handlers[i].context == context))
        {
            return HAL_OK;
        }
    }
    if (bus->handler_count >= DM_2006_BUS_MAX_HANDLERS)
    {
        return HAL_ERROR;
    }

    bus->handlers[bus->handler_count].callback = callback;
    bus->handlers[bus->handler_count].context = context;
    bus->handler_count++;
    return HAL_OK;
}

HAL_StatusTypeDef DM2006_BusSend(dm_2006_bus_t *bus, uint16_t id,
                                 const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef header = {0};
    HAL_StatusTypeDef status;

    if ((bus == NULL) || (bus->ready == 0U) || (data == NULL) ||
        (id > DM_2006_BUS_STD_ID_MAX))
    {
        return HAL_ERROR;
    }
    if (HAL_FDCAN_GetTxFifoFreeLevel(bus->device) == 0U)
    {
        bus->diag.tx_busy_count++;
        return HAL_BUSY;
    }

    header.Identifier = id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    status = HAL_FDCAN_AddMessageToTxFifoQ(bus->device, &header, data);
    if (status == HAL_OK)
    {
        bus->diag.tx_count++;
    }
    else
    {
        bus->diag.tx_error_count++;
    }

    return status;
}

void DM2006_BusHandleRxInterrupt(dm_2006_bus_t *bus,
                                 uint32_t interrupt_flags)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if ((bus == NULL) || (bus->ready == 0U) ||
        ((interrupt_flags & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U))
    {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(bus->device, FDCAN_RX_FIFO0) > 0U)
    {
        uint8_t head;
        uint8_t next;

        if (HAL_FDCAN_GetRxMessage(bus->device, FDCAN_RX_FIFO0, &header,
                                   data) != HAL_OK)
        {
            bus->diag.rx_error_count++;
            return;
        }
        if ((header.IdType != FDCAN_STANDARD_ID) ||
            (header.RxFrameType != FDCAN_DATA_FRAME) ||
            (header.DataLength != FDCAN_DLC_BYTES_8))
        {
            continue;
        }

        head = bus->rx_head;
        next = (uint8_t)((head + 1U) & DM_2006_BUS_RX_QUEUE_MASK);
        if (next == bus->rx_tail)
        {
            bus->diag.rx_overflow_count++;
            continue;
        }

        bus->rx_queue[head].id = (uint16_t)header.Identifier;
        bus->rx_queue[head].tick_ms = HAL_GetTick();
        memcpy(bus->rx_queue[head].data, data, sizeof(data));
        __DMB();
        bus->rx_head = next;
        bus->diag.rx_count++;
    }
}

void DM2006_BusHandleErrorInterrupt(dm_2006_bus_t *bus,
                                    uint32_t interrupt_flags)
{
    if ((bus == NULL) || (bus->ready == 0U))
    {
        return;
    }
    if ((interrupt_flags & FDCAN_IT_ERROR_WARNING) != 0U)
    {
        bus->diag.error_warning_count++;
    }
    if ((interrupt_flags & FDCAN_IT_ERROR_PASSIVE) != 0U)
    {
        bus->diag.error_passive_count++;
    }
    if ((interrupt_flags & FDCAN_IT_BUS_OFF) != 0U)
    {
        bus->diag.bus_off_count++;
    }
}

void DM2006_BusProcessRx(dm_2006_bus_t *bus)
{
    if ((bus == NULL) || (bus->ready == 0U))
    {
        return;
    }

    while (bus->rx_tail != bus->rx_head)
    {
        dm_2006_bus_frame_t frame;
        uint8_t tail = bus->rx_tail;
        uint8_t i;

        frame = bus->rx_queue[tail];
        __DMB();
        bus->rx_tail = (uint8_t)((tail + 1U) & DM_2006_BUS_RX_QUEUE_MASK);

        for (i = 0U; i < bus->handler_count; i++)
        {
            bus->handlers[i].callback(bus->handlers[i].context, frame.id,
                                      frame.data, frame.tick_ms);
        }
    }
}

void DM2006_BusGetDiag(const dm_2006_bus_t *bus,
                       dm_2006_bus_diag_t *diag)
{
    if ((bus != NULL) && (diag != NULL))
    {
        *diag = bus->diag;
    }
}
