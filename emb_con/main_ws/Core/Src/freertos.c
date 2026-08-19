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

#include "uart_interact.h"

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

extern Chassis ch;
extern M3508_CAN_All m3508;
extern Chassis_Task1 t1;
extern Chassis_Config cfg;
extern UartInteract it;


/* USER CODE END Variables */
/* Definitions for ChassisMainTask */
osThreadId_t ChassisMainTaskHandle;
const osThreadAttr_t ChassisMainTask_attributes = {
  .name = "ChassisMainTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Comms_Task */
osThreadId_t Comms_TaskHandle;
const osThreadAttr_t Comms_Task_attributes = {
  .name = "Comms_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CmdQueueTask1 */
osMessageQueueId_t CmdQueueTask1Handle;
const osMessageQueueAttr_t CmdQueueTask1_attributes = {
  .name = "CmdQueueTask1"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartChassisBaseTask(void *argument);
void StartComms_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* Create the queue(s) */
  /* creation of CmdQueueTask1 */
  CmdQueueTask1Handle = osMessageQueueNew (32, sizeof(uint8_t), &CmdQueueTask1_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ChassisMainTask */
  ChassisMainTaskHandle = osThreadNew(StartChassisBaseTask, NULL, &ChassisMainTask_attributes);

  /* creation of Comms_Task */
  Comms_TaskHandle = osThreadNew(StartComms_Task, NULL, &Comms_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartChassisBaseTask */
/**
  * @brief  Function implementing the ChassisMainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartChassisBaseTask */
void StartChassisBaseTask(void *argument)
{
  /* USER CODE BEGIN StartChassisBaseTask */
  /* Infinite loop */
  for(;;)
  {
    osThreadFlagsWait(0x01u, osFlagsWaitAny, osWaitForever);
    // if (UartInteract_IsPaused(&it)) {
    if (it.task_paused) {
      /* 手动接管：只跑底盘闭环，跳过任务1 FSM，避免自动逻辑和手控打架 */
      UartInteract_CheckVelocityTimeout(&it, HAL_GetTick());
      Chassis_Update(&ch);
    } else {
      Chassis_Task1_Update(&t1);
    }
    // osDelay(1);
  }
  /* USER CODE END StartChassisBaseTask */
}

/* USER CODE BEGIN Header_StartComms_Task */
/**
* @brief Function implementing the Comms_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartComms_Task */
void StartComms_Task(void *argument)
{
  /* USER CODE BEGIN StartComms_Task */
  uint8_t rx_byte = 0;
  /* Infinite loop */
  for(;;)
  {
    if (osMessageQueueGet(CmdQueueTask1Handle, &rx_byte, NULL, 10U) == osOK) {
      UartInteract_RxByte(&it, rx_byte);
      while (osMessageQueueGet(CmdQueueTask1Handle, &rx_byte, NULL, 0U) == osOK) {
        UartInteract_RxByte(&it, rx_byte);   // 把一帧的字节在本次循环凑齐
      }
    }
    UartInteract_Poll(&it);
    // osDelay(1);
  }
  /* USER CODE END StartComms_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

