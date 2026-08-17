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
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_app.h"
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motorPIDTASK */
osThreadId_t motorPIDTASKHandle;
const osThreadAttr_t motorPIDTASK_attributes = {
  .name = "motorPIDTASK",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for motorCurTask */
osThreadId_t motorCurTaskHandle;
const osThreadAttr_t motorCurTask_attributes = {
  .name = "motorCurTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for motorDataMutex */
osMutexId_t motorDataMutexHandle;
const osMutexAttr_t motorDataMutex_attributes = {
  .name = "motorDataMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartMotorPIDTask(void *argument);
void StartMotorCurTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of motorDataMutex */
  motorDataMutexHandle = osMutexNew(&motorDataMutex_attributes);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of motorPIDTASK */
  motorPIDTASKHandle = osThreadNew(StartMotorPIDTask, NULL, &motorPIDTASK_attributes);

  /* creation of motorCurTask */
  motorCurTaskHandle = osThreadNew(StartMotorCurTask, NULL, &motorCurTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartMotorPIDTask */
/**
* @brief Function implementing the motorPIDTASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorPIDTask */
void StartMotorPIDTask(void *argument)
{
  /* USER CODE BEGIN StartMotorPIDTask */
  /* Infinite loop */
  for(;;)
  {
    osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
    osMutexAcquire(motorDataMutexHandle, osWaitForever);
    Motor_PIDUpdate();
    //VOFA_Send();  // 需要遥测时取消注释
    osMutexRelease(motorDataMutexHandle);
  }
  /* USER CODE END StartMotorPIDTask */
}

/* USER CODE BEGIN Header_StartMotorCurTask */
/**
* @brief Function implementing the motorCurTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorCurTask */
void StartMotorCurTask(void *argument)
{
  /* USER CODE BEGIN StartMotorCurTask */
  /* Infinite loop */
  for(;;)
  {
    osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);// 等待 TIM1 1kHz 中断触发
    osMutexAcquire(motorDataMutexHandle, osWaitForever);
    Motor_CurUpdate();
    osMutexRelease(motorDataMutexHandle);

    // VOFA_Send();  // 需要遥测时取消注释
  }
  /* USER CODE END StartMotorCurTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// TIM1/TIM2 中断 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {        // TIM1 1kHz -> 发电流任务
        if (motorCurTaskHandle != NULL) {
            osThreadFlagsSet(motorCurTaskHandle, 0x01);
        }
    } else if (htim->Instance == TIM2) { // TIM2 500Hz -> PID 任务
        if (motorPIDTASKHandle != NULL) {
            osThreadFlagsSet(motorPIDTASKHandle, 0x01);
        }
    }
}

/* USER CODE END Application */

