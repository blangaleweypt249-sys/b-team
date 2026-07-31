#ifndef FREERTOS_APP_H
#define FREERTOS_APP_H

#include <stdint.h>

extern volatile uint32_t freertos_app_task_stack_free_bytes;
extern volatile uint32_t freertos_comm_task_stack_free_bytes;
extern volatile uint32_t freertos_monitor_task_stack_free_bytes;
extern volatile uint32_t freertos_heap_free_bytes;
extern volatile uint32_t freertos_heap_min_ever_free_bytes;
extern volatile uint32_t freertos_malloc_failed_count;
extern volatile const char *freertos_stack_overflow_task_name;

void App_Periodic10ms(void);
void App_Control1ms(void);

#endif
