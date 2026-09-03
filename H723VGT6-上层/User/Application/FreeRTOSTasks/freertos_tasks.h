/**
 * @file freertos_tasks.h
 * @brief 声明上层应用控制任务和通信接收任务的入口函数。
 */

#ifndef FREERTOS_TASKS_H
#define FREERTOS_TASKS_H /**< 防止 freertos_tasks.h 被重复包含。 */

#ifdef __cplusplus
extern "C" {
#endif

/* 功能：声明上层应用周期任务入口；参数 argument 由 RTOS 创建任务时传入。 */
extern void StartAppTask(void *argument /**< RTOS 创建任务时传入的任务参数 */);
/* 功能：声明通信接收处理任务入口；参数 argument 由 RTOS 创建任务时传入。 */
extern void StartCommRxTask(void *argument /**< RTOS 创建任务时传入的任务参数 */);

#ifdef __cplusplus
}
#endif

#endif /* FreeRTOS 任务头文件保护宏 */
