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
#include "m3508_driver.h"
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
volatile float g_arm_distance_mm = 0.0f;   /* 测试：正负=方向 */
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
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motorDataMutex */
osMutexId_t motorDataMutexHandle;
const osMutexAttr_t motorDataMutex_attributes = {
  .name = "motorDataMutex"
};
/* Definitions for armTask */
osThreadId_t armTaskHandle;
const osThreadAttr_t armTask_attributes = {
  .name = "armTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osSemaphoreId_t armSemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void SimVision_GiveMove(float distance_mm);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartMotorPIDTask(void *argument);
void StartMotorCurTask(void *argument);
void StartArmTask(void *argument);

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

  armSemHandle = osSemaphoreNew(1, 0, NULL);   /* 二进制信号量，初始0 → 无信号量=永久挂起 */

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
  /* 机械臂任务：上电演示一轮“正转→锁定→回位→锁死”，之后保持锁定 */
  armTaskHandle = osThreadNew(StartArmTask, NULL, &armTask_attributes);
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
	  SimVision_GiveMove( 400.0f);    /* 模拟视觉：正转 400mm */
	      vTaskDelay(pdMS_TO_TICKS(4000)); /* 等动作完成 + 锁死停留 */

	      SimVision_GiveMove(-400.0f);    /* 模拟视觉：反转回 400mm */
	      vTaskDelay(pdMS_TO_TICKS(4000));
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
    VOFA_Send();  // 需要遥测时取消注释
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

     VOFA_Send();  // 需要遥测时取消注释
  }
  /* USER CODE END StartMotorCurTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* 模拟视觉模块：这就是将来 uart_interact.c 的 OnFrame case 里要调的两行。
 * 正式接视觉时，删掉本函数，让 UART 回调直接调这两行即可。 */
static void SimVision_GiveMove(float distance_mm)
{
    g_arm_distance_mm = distance_mm;      /* 写距离（float，32位原子写） */
    osSemaphoreRelease(armSemHandle);     /* 给信号量（ISR 安全） —— 外部"Release" */
}



/* 机械臂任务：调用 Arm_MoveLock（distance_mm 正负=正转/反转；移动 |mm| 后锁死） */
void StartArmTask(void *argument)
{
    /* 等系统稳定、等 C620 完成上电自检 */
    while (!M3508_IsFeedbackReady()) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    /* 无信号量 → 永久挂起；每次收到 Give → 执行一次移动（锁死保持到下次） */
    for (;;)
    {
        osSemaphoreAcquire(armSemHandle, osWaitForever);   /* 阻塞等信号量 */
        Arm_MoveLock((double)g_arm_distance_mm);           /* 一次动作 */
    }
}

// TIM1/TIM2 中断
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {        // TIM1 1kHz -> 发电流任务
        if (motorCurTaskHandle != NULL) {
            osThreadFlagsSet(motorCurTaskHandle, 0x01);
        }
    } else if (htim->Instance == TIM2) {
        if (motorPIDTASKHandle != NULL) {
            osThreadFlagsSet(motorPIDTASKHandle, 0x01);
        }
    }
}



/* USER CODE END Application */

