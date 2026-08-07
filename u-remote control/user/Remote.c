#include "Remote.h"
#include "adc.h"

#define REMOTE_ADC1_NUM  2U  // ADC1 DMA采样通道数
#define REMOTE_ADC2_NUM  4U  // ADC2 DMA采样通道数

static volatile uint16_t remote_adc1_value[REMOTE_ADC1_NUM];
static volatile uint16_t remote_adc2_value[REMOTE_ADC2_NUM];

volatile remote_data_t remote_data;
uint8_t remote_adc_error;

/**
 * @brief 启动遥控器ADC的循环DMA采样
 * @param None
 * @retval None
 */
void Remote_Init(void)
{
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)remote_adc1_value, REMOTE_ADC1_NUM) != HAL_OK)
    {
        remote_adc_error = 1;
        return;
    }

    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)remote_adc2_value, REMOTE_ADC2_NUM) != HAL_OK)
    {
        remote_adc_error = 1;
        return;
    }
}

/**
 * @brief 更新遥控器六路ADC数据
 * @param None
 * @retval None
 */
void Remote_Update(void)
{
    if (remote_adc_error != 0)
    {
        return;
    }

    remote_data.left_shoulder = remote_adc1_value[0];
    remote_data.right_shoulder = remote_adc1_value[1];
    remote_data.left_x = remote_adc2_value[0];
    remote_data.left_y = remote_adc2_value[1];
    remote_data.right_x = remote_adc2_value[2];
    remote_data.right_y = remote_adc2_value[3];
}
