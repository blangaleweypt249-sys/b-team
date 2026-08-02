#include "fdcan_task.h"

#include "fdcan.h"

#include <stdbool.h>
#include <stddef.h>

static rs_bus_t rs_bus;
static dm_2006_bus_t dm_2006_bus;
static bool task_ready;

HAL_StatusTypeDef FDCAN_TaskInit(const fdcan_task_config_t *config)
{
    if (task_ready)
    {
        return HAL_OK;
    }
    if ((config == NULL) || (config->dm_2006_rx_ids == NULL) ||
        (config->dm_2006_rx_id_count == 0U))
    {
        return HAL_ERROR;
    }

    if (RS_BusInit(&rs_bus, &hfdcan2, config->rs_host_id) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (DM2006_BusInit(&dm_2006_bus, &hfdcan3,
                       config->dm_2006_rx_ids,
                       config->dm_2006_rx_id_count) != HAL_OK)
    {
        RS_BusStop(&rs_bus);
        return HAL_ERROR;
    }

    task_ready = true;
    return HAL_OK;
}

void FDCAN_TaskRun1ms(void)
{
    if (!task_ready)
    {
        return;
    }

    RS_BusProcessRx(&rs_bus);
    DM2006_BusProcessRx(&dm_2006_bus);
}

void FDCAN_TaskStop(void)
{
    if (!task_ready)
    {
        return;
    }

    RS_BusStop(&rs_bus);
    DM2006_BusStop(&dm_2006_bus);
    task_ready = false;
}

rs_bus_t *FDCAN_TaskGetRsBus(void)
{
    return task_ready ? &rs_bus : NULL;
}

dm_2006_bus_t *FDCAN_TaskGetDm2006Bus(void)
{
    return task_ready ? &dm_2006_bus : NULL;
}

void FDCAN_TaskHandleRxInterrupt(FDCAN_HandleTypeDef *hfdcan,
                                 uint32_t interrupt_flags)
{
    if (hfdcan == &hfdcan2)
    {
        RS_BusHandleRxInterrupt(&rs_bus, interrupt_flags);
    }
    else if (hfdcan == &hfdcan3)
    {
        DM2006_BusHandleRxInterrupt(&dm_2006_bus, interrupt_flags);
    }
}

void FDCAN_TaskHandleErrorInterrupt(FDCAN_HandleTypeDef *hfdcan,
                                    uint32_t interrupt_flags)
{
    if (hfdcan == &hfdcan3)
    {
        DM2006_BusHandleErrorInterrupt(&dm_2006_bus, interrupt_flags);
    }
}
