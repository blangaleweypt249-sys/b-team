#ifndef DT35_H
#define DT35_H

#include "stm32f1xx_hal.h"


#define DT35_I2C_ADDR          0x41U

#define DT35_VOLT_NEAR_MV      0U
#define DT35_VOLT_FAR_MV       10000U
#define DT35_DIST_NEAR_CM      5.0f
#define DT35_DIST_FAR_CM       20.0f

typedef struct
{
    uint16_t raw;
    uint16_t voltage_mv;
    float distance_cm;
} DT35_Data;

HAL_StatusTypeDef DT35_Init(I2C_HandleTypeDef *i2c);
HAL_StatusTypeDef DT35_Read(void);
HAL_StatusTypeDef DT35_Get(DT35_Data *data);

#endif
