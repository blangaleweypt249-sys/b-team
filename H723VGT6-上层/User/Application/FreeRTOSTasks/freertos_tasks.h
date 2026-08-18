#ifndef FREERTOS_TASKS_H
#define FREERTOS_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

extern void StartAppTask(void *argument);
extern void StartCommRxTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* FreeRTOS 任务头文件保护宏 */
