#ifndef FDCAN_TASK_H
#define FDCAN_TASK_H

#include "dm_2006_bus.h"
#include "rs_bus.h"

#include <stdint.h>

typedef struct
{
    uint8_t rs_host_id;
    const uint16_t *dm_2006_rx_ids;
    uint8_t dm_2006_rx_id_count;
} fdcan_task_config_t;

HAL_StatusTypeDef FDCAN_TaskInit(const fdcan_task_config_t *config);
void FDCAN_TaskRun1ms(void);
void FDCAN_TaskStop(void);
rs_bus_t *FDCAN_TaskGetRsBus(void);
dm_2006_bus_t *FDCAN_TaskGetDm2006Bus(void);
void FDCAN_TaskHandleRxInterrupt(FDCAN_HandleTypeDef *hfdcan,
                                 uint32_t interrupt_flags);
void FDCAN_TaskHandleErrorInterrupt(FDCAN_HandleTypeDef *hfdcan,
                                    uint32_t interrupt_flags);

#endif
