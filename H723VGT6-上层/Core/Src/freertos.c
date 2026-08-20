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
#include "freertos_tasks.h"
#include "upper_entry.h"
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
volatile uint32_t freertos_malloc_failed_count;
volatile const char *freertos_stack_overflow_task_name;
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
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartAppTask(void *argument);
void StartCommRxTask(void *argument);

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

    (void)CommRuntime_PcTransmitCopy(init_error,
                                     sizeof(init_error) - 1U);
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

  /* USER CODE BEGIN RTOS_THREADS */
  if ((appTaskHandle == NULL) ||
      (commRxTaskHandle == NULL))
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
__weak void StartAppTask(void *argument)
{
  /* USER CODE BEGIN StartAppTask */
  (void)argument;

  /* The strong implementation is provided by freertos_tasks.c. */
  for (;;)
  {
    (void)osDelay(osWaitForever);
  }
  /* USER CODE END StartAppTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
__weak void App_Control1ms(void)
{
}
/* USER CODE END Application */

