#ifndef DT35_PNP_LINK_H
#define DT35_PNP_LINK_H

#include "stm32h7xx_hal.h"

/* DT35 on I2C2: front=1000001, left=1000000. */
#define DT35_LINK_ADDR_LEFT     0x40U
#define DT35_LINK_ADDR_FRONT    0x41U
#define DT35_LINK_LEFT_INDEX    0U
#define DT35_LINK_FRONT_INDEX   1U

/* PNP upper-computer names: left=B/1000001, right=F/1000000. */
#define PNP_LINK_ADDR_RIGHT_F   0x40U
#define PNP_LINK_ADDR_LEFT_B    0x41U
#define PNP_LINK_RIGHT_INDEX    0U
#define PNP_LINK_LEFT_INDEX     1U
#define SENSOR_LINK_COUNT       2U

/*
 * DT35 frame: AA, address, distance_cm low, high, XOR checksum.
 * PNP frame:  AB, address, trigger low, 0, XOR checksum.
 */

typedef struct
{
    uint32_t last_rx_ms;
    uint16_t distance_cm;
    uint8_t online;
    uint8_t frame_pending;
} dt35_link_t;

typedef struct
{
    uint32_t last_rx_ms;
    uint8_t trigger;
    uint8_t online;
    uint8_t frame_pending;
} pnp_link_t;

extern volatile dt35_link_t dt35_link[SENSOR_LINK_COUNT];
extern volatile pnp_link_t pnp_link[SENSOR_LINK_COUNT];

/** Initialize the DT35/PNP UART link. */
HAL_StatusTypeDef DT35PnpLink_Init(UART_HandleTypeDef *uart);
/** Recover UART reception and expire stale sensor data. */
void DT35PnpLink_Run(void);
/** Forward pending DT35/PNP telemetry to the upper computer. */
void DT35PnpLink_Send(UART_HandleTypeDef *uart);
/** Handle completion of one sensor-UART byte. */
void DT35PnpLink_RxCplt(UART_HandleTypeDef *uart);
/** Request UART reception recovery after an error. */
void DT35PnpLink_Error(UART_HandleTypeDef *uart);

#endif
