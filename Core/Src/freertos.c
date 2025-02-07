/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "SEGGER_SYSVIEW_FreeRTOS.h"

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
osThreadId Start_INS_TaskHandle;
uint32_t Start_INS_TaskBuffer[ 1024 ];
osStaticThreadDef_t Start_INS_TaskControlBlock;
osThreadId Start_Control_TaskHandle;
uint32_t Start_Control_TaskBuffer[ 1024 ];
osStaticThreadDef_t Start_Control_TaskControlBlock;
osThreadId Start_CAN_TaskHandle;
uint32_t Start_CAN_TaskBuffer[ 1024 ];
osStaticThreadDef_t Start_CAN_TaskControlBlock;
osThreadId Start_Detect_TaskHandle;
uint32_t Start_Detect_TaskBuffer[ 1024 ];
osStaticThreadDef_t Start_Detect_TaskControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void INS_Task(void const * argument);
void Control_Task(void const * argument);
void CAN_Task(void const * argument);
void Detect_Task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

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
  /* definition and creation of Start_INS_Task */
  osThreadStaticDef(Start_INS_Task, INS_Task, osPriorityHigh, 0, 1024, Start_INS_TaskBuffer, &Start_INS_TaskControlBlock);
  Start_INS_TaskHandle = osThreadCreate(osThread(Start_INS_Task), NULL);

  /* definition and creation of Start_Control_Task */
  osThreadStaticDef(Start_Control_Task, Control_Task, osPriorityAboveNormal, 0, 1024, Start_Control_TaskBuffer, &Start_Control_TaskControlBlock);
  Start_Control_TaskHandle = osThreadCreate(osThread(Start_Control_Task), NULL);

  /* definition and creation of Start_CAN_Task */
  osThreadStaticDef(Start_CAN_Task, CAN_Task, osPriorityNormal, 0, 1024, Start_CAN_TaskBuffer, &Start_CAN_TaskControlBlock);
  Start_CAN_TaskHandle = osThreadCreate(osThread(Start_CAN_Task), NULL);

  /* definition and creation of Start_Detect_Task */
  osThreadStaticDef(Start_Detect_Task, Detect_Task, osPriorityBelowNormal, 0, 1024, Start_Detect_TaskBuffer, &Start_Detect_TaskControlBlock);
  Start_Detect_TaskHandle = osThreadCreate(osThread(Start_Detect_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_INS_Task */
/**
  * @brief  Function implementing the StartINS thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_INS_Task */
__weak void INS_Task(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN INS_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INS_Task */
}

/* USER CODE BEGIN Header_Control_Task */
/**
* @brief Function implementing the StartControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Control_Task */
__weak void Control_Task(void const * argument)
{
  /* USER CODE BEGIN Control_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Control_Task */
}

/* USER CODE BEGIN Header_CAN_Task */
/**
* @brief Function implementing the StartCAN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Task */
__weak void CAN_Task(void const * argument)
{
  /* USER CODE BEGIN CAN_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CAN_Task */
}

/* USER CODE BEGIN Header_Detect_Task */
/**
* @brief Function implementing the Start_Detect_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Detect_Task */
__weak void Detect_Task(void const * argument)
{
  /* USER CODE BEGIN Detect_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Detect_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
