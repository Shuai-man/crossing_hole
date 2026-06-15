/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "Offline_Task.h"
#include "RefereeTask.h"
#include "PowerControlTask.h"
#include "GimbalTask.h"
#include "BlueToothTask.h"
#include "ChassisControlTask.h"

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

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId offline_taskHandle;
osThreadId gimbalTaskHandle;
osThreadId powerControlTasHandle;
osThreadId refereetaskHandle;
osThreadId _ChassisCtrlHandle;
osThreadId BlueToothHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Offline_task(void const * argument);
void GimbalTask(void const * argument);
void PowerControlTask(void const * argument);
void Refereetask(void const * argument);
void ChassisControl_task(void const * argument);
void BlueToothTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of offline_task */
  osThreadDef(offline_task, Offline_task, osPriorityRealtime, 0, 1024);
  offline_taskHandle = osThreadCreate(osThread(offline_task), NULL);

  /* definition and creation of gimbalTask */
  osThreadDef(gimbalTask, GimbalTask, osPriorityRealtime, 0, 1024);
  gimbalTaskHandle = osThreadCreate(osThread(gimbalTask), NULL);

  /* definition and creation of powerControlTas */
  osThreadDef(powerControlTas, PowerControlTask, osPriorityRealtime, 0, 1024);
  powerControlTasHandle = osThreadCreate(osThread(powerControlTas), NULL);

  /* definition and creation of refereetask */
  osThreadDef(refereetask, Refereetask, osPriorityRealtime, 0, 1024);
  refereetaskHandle = osThreadCreate(osThread(refereetask), NULL);

  /* definition and creation of _ChassisCtrl */
  osThreadDef(_ChassisCtrl, ChassisControl_task, osPriorityRealtime, 0, 1024);
  _ChassisCtrlHandle = osThreadCreate(osThread(_ChassisCtrl), NULL);

  /* definition and creation of BlueTooth */
  osThreadDef(BlueTooth, BlueToothTask, osPriorityRealtime, 0, 1024);
  BlueToothHandle = osThreadCreate(osThread(BlueTooth), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Offline_task */
/**
* @brief Function implementing the offline_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Offline_task */
__weak void Offline_task(void const * argument)
{
  /* USER CODE BEGIN Offline_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Offline_task */
}

/* USER CODE BEGIN Header_GimbalTask */
/**
* @brief Function implementing the gimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GimbalTask */
__weak void GimbalTask(void const * argument)
{
  /* USER CODE BEGIN GimbalTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END GimbalTask */
}

/* USER CODE BEGIN Header_PowerControlTask */
/**
* @brief Function implementing the powerControlTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PowerControlTask */
__weak void PowerControlTask(void const * argument)
{
  /* USER CODE BEGIN PowerControlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END PowerControlTask */
}

/* USER CODE BEGIN Header_Refereetask */
/**
* @brief Function implementing the refereetask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Refereetask */
__weak void Refereetask(void const * argument)
{
  /* USER CODE BEGIN Refereetask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Refereetask */
}

/* USER CODE BEGIN Header_ChassisControl_task */
/**
* @brief Function implementing the _ChassisControls thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassisControl_task */
__weak void ChassisControl_task(void const * argument)
{
  /* USER CODE BEGIN ChassisControl_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ChassisControl_task */
}

/* USER CODE BEGIN Header_BlueToothTask */
/**
* @brief Function implementing the _BlueToothTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BlueToothTask */
__weak void BlueToothTask(void const * argument)
{
  /* USER CODE BEGIN BlueToothTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END BlueToothTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
