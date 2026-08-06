#include "mymain.h"
#include "i2c.h"

#define SAMPLE_TIME_MS         50U
#define RETRY_TIME_MS          1000U

DT35_Data dt35_data;
HAL_StatusTypeDef dt35_state = HAL_ERROR;

static uint32_t sample_tick;
static uint32_t retry_tick;
static uint8_t read_busy;

void MyMain_Init(void)
{
    dt35_state = DT35_Init(&hi2c2);
    sample_tick = HAL_GetTick();
    retry_tick = sample_tick;
    read_busy = 0U;
}

void MyMain_Loop(void)
{
    uint32_t now;
    DT35_Data data;

    now = HAL_GetTick();

    if (read_busy != 0U)
    {
        dt35_state = DT35_Get(&data);
        if (dt35_state == HAL_BUSY)
        {
            dt35_state = HAL_OK;
            return;
        }

        read_busy = 0U;
        if (dt35_state == HAL_OK)
        {
            dt35_data = data;
        }
        else
        {
            retry_tick = now;
        }
        return;
    }

    if (dt35_state != HAL_OK)
    {
        if ((uint32_t)(now - retry_tick) >= RETRY_TIME_MS)
        {
            retry_tick = now;
            dt35_state = DT35_Init(&hi2c2);
            sample_tick = HAL_GetTick();
            read_busy = 0U;
        }
        return;
    }

    if ((uint32_t)(now - sample_tick) < SAMPLE_TIME_MS)
    {
        return;
    }
    sample_tick = now;

    dt35_state = DT35_Read();
    if (dt35_state == HAL_OK)
    {
        read_busy = 1U;
    }
    else
    {
        retry_tick = now;
    }
}
