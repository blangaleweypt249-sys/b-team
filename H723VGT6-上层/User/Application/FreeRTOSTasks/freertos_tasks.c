#include "freertos_tasks.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"

#include "comm_runtime.h"
#include "freertos_app.h"
#include "W25Qxx.h"

extern osThreadId_t commRxTaskHandle;

volatile w25q_status_t w25q_init_status = W25Q_ERROR_NOT_INIT;

void StartAppTask(void *argument)
{
  uint32_t next_wake;
  uint32_t period_10ms;

  (void)argument;
  next_wake = osKernelGetTickCount();
  period_10ms = 0U;

  for (;;)
  {
    next_wake += 1U;
    App_Control1ms();
    period_10ms++;
    if (period_10ms >= 10U)
    {
      period_10ms = 0U;
      App_Periodic10ms();
    }
    (void)osDelayUntil(next_wake);
  }
}

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
