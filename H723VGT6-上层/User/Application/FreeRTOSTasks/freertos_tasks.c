/**
 * @file freertos_tasks.c
 * @brief 实现上层控制周期任务和通信接收处理任务。
 */

#include "freertos_tasks.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"

#include "comm_runtime.h"
#include "freertos_app.h"
#include "W25Qxx.h"

extern osThreadId_t commRxTaskHandle;

volatile w25q_status_t w25q_init_status = W25Q_ERROR_NOT_INIT;

/* 功能：运行上层 1 ms 控制任务；参数 argument 为 RTOS 任务入口参数。 */
void StartAppTask(void *argument)
{
  uint32_t next_wake;

  (void)argument;
  next_wake = osKernelGetTickCount();

  for (;;)
  {
    next_wake += 1U;
    App_Control1ms();
    (void)osDelayUntil(next_wake);
  }
}

/* 功能：初始化 Flash 与通信运行时并持续处理接收事件；参数 argument 为 RTOS 任务入口参数；发生初始化错误时进入错误处理。 */
void StartCommRxTask(void *argument)
{
  uint32_t flags;
  HAL_StatusTypeDef comm_status;

  (void)argument;
  w25q_init_status = W25Q_PortInit();
  comm_status = CommRuntime_Init(commRxTaskHandle);
  if (comm_status != HAL_OK)
  {
    Error_Handler();
  }

  for (;;)
  {
    flags = osThreadFlagsWait(COMM_EVENT_ALL, osFlagsWaitAny, osWaitForever);
    if ((flags & osFlagsError) == 0U)
    {
      CommRuntime_Process(flags);
    }
  }
}
