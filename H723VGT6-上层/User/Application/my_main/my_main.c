/**
 * @file my_main.c
 * @brief 实现用户应用的基础硬件初始化入口。
 */

#include "my_main.h"

/* 功能：初始化用户自定义硬件状态；当前用于设置状态指示灯的初始电平。 */
void My_Init(void)
{
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
}
