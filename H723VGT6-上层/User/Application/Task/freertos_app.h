/**
 * @file freertos_app.h
 * @brief 声明 FreeRTOS 应用周期入口和任务栈监测变量。
 */

#ifndef FREERTOS_APP_H
#define FREERTOS_APP_H

#include <stdint.h>

extern volatile uint32_t freertos_malloc_failed_count;
extern volatile const char *freertos_stack_overflow_task_name;

/* 功能：声明上层应用的 1 ms 控制入口。 */
/* 功能：使用 HAL 毫秒时基调用上层控制周期；用途：提供给系统任务的固定入口；无返回值表示完成本次 1 ms 调用。 */
void App_Control1ms(void);

#endif
