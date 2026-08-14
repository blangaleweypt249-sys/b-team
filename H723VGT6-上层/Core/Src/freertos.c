/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "comm_runtime.h"
#include "freertos_app.h"
#include "upper_entry.h"
#include "W25Qxx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile uint32_t freertos_app_task_stack_free_bytes;
volatile uint32_t freertos_comm_task_stack_free_bytes;
volatile uint32_t freertos_monitor_task_stack_free_bytes;
volatile uint32_t freertos_heap_free_bytes;
volatile uint32_t freertos_heap_min_ever_free_bytes;
volatile uint32_t freertos_malloc_failed_count;
volatile const char *freertos_stack_overflow_task_name;
volatile w25q_status_t w25q_init_status = W25Q_ERROR_NOT_INIT;
/* USER CODE END Variables */
/* Definitions for appTask */
osThreadId_t appTaskHandle;
const osThreadAttr_t appTask_attributes = {
  .name = "appTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for commRxTask */
osThreadId_t commRxTaskHandle;
const osThreadAttr_t commRxTask_attributes = {
  .name = "commRxTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for monitorTask */
osThreadId_t monitorTaskHandle;
const osThreadAttr_t monitorTask_attributes = {
  .name = "monitorTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartAppTask(void *argument);
void StartCommRxTask(void *argument);
void StartMonitorTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName)
{
  (void)xTask;
  freertos_stack_overflow_task_name = pcTaskName;
  Error_Handler();
}

void vApplicationMallocFailedHook(void)
{
  freertos_malloc_failed_count++;
  Error_Handler();
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  if (!UpperEntry_Init())
  {
    static const uint8_t init_error[] = "FLASH BOOT ERROR UPPER_INIT\r\n";

    (void)CommRuntime_PcTransmitBlocking(init_error,
                                         sizeof(init_error) - 1U,
                                         500U);
    Error_Handler();
  }
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of appTask */
  appTaskHandle = osThreadNew(StartAppTask, NULL, &appTask_attributes);

  /* creation of commRxTask */
  commRxTaskHandle = osThreadNew(StartCommRxTask, NULL, &commRxTask_attributes);

  /* creation of monitorTask */
  monitorTaskHandle = osThreadNew(StartMonitorTask, NULL, &monitorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  if ((appTaskHandle == NULL) ||
      (commRxTaskHandle == NULL) ||
      (monitorTaskHandle == NULL))
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartAppTask */
/**
  * @brief  Function implementing the appTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartAppTask */
void StartAppTask(void *argument)
{
  /* USER CODE BEGIN StartAppTask */
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
  /* USER CODE END StartAppTask */
}

/* USER CODE BEGIN Header_StartCommRxTask */
/**
* @brief Function implementing the commRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommRxTask */
void StartCommRxTask(void *argument)
{
  /* USER CODE BEGIN StartCommRxTask */
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
  /* USER CODE END StartCommRxTask */
}

/* USER CODE BEGIN Header_StartMonitorTask */
/**
* @brief Function implementing the monitorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMonitorTask */
void StartMonitorTask(void *argument)
{
  /* USER CODE BEGIN StartMonitorTask */
  uint32_t next_wake;

  (void)argument;
  next_wake = osKernelGetTickCount();
  for (;;)
  {
    next_wake += 1000U;
    freertos_app_task_stack_free_bytes =
      uxTaskGetStackHighWaterMark((TaskHandle_t)appTaskHandle) * sizeof(StackType_t);
    freertos_comm_task_stack_free_bytes =
      uxTaskGetStackHighWaterMark((TaskHandle_t)commRxTaskHandle) * sizeof(StackType_t);
    freertos_monitor_task_stack_free_bytes =
      uxTaskGetStackHighWaterMark((TaskHandle_t)monitorTaskHandle) * sizeof(StackType_t);
    freertos_heap_free_bytes = xPortGetFreeHeapSize();
    freertos_heap_min_ever_free_bytes = xPortGetMinimumEverFreeHeapSize();
    (void)osDelayUntil(next_wake);
  }
  /* USER CODE END StartMonitorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
__weak void App_Periodic10ms(void)
{
}

__weak void App_Control1ms(void)
{
}
/* USER CODE END Application */

