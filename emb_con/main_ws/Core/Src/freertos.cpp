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

extern "C" {

#include "uart_interact.h"
#include "app.h"
#include "chassis.h"
#include "chassis_task_1.h"
#include "m3508_driver.h"
#include "tim.h"

}

#include "task1.hpp"

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

extern App_Context global_app;

uint8_t App_IsPaused(void);
void App_CheckVelocityTimeout(uint32_t now_ms);
void App_PollEvents(void);

volatile uint8_t arm_flag1 = 0;  // 暂时使用的标志位

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
/* Definitions for MechaArm_Task */
osThreadId_t MechaArm_TaskHandle;
const osThreadAttr_t MechaArm_Task_attributes = {
  .name = "MechaArm_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CmdQueueTask1 */
osMessageQueueId_t CmdQueueTask1Handle;
const osMessageQueueAttr_t CmdQueueTask1_attributes = {
  .name = "CmdQueueTask1"
};
/* Definitions for Chassis_M3508 */
osTimerId_t Chassis_M3508Handle;
const osTimerAttr_t Chassis_M3508_attributes = {
  .name = "Chassis_M3508"
};
/* Definitions for Task1_ArmGet */
osSemaphoreId_t Task1_ArmGetHandle;
const osSemaphoreAttr_t Task1_ArmGet_attributes = {
  .name = "Task1_ArmGet"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

extern "C" void StartChassisBaseTask(void *argument);
extern "C" void StartComms_Task(void *argument);
extern "C" void StartMechaArm_Task(void *argument);
extern "C" void Chassis_M3508Callback(void *argument);

extern "C" void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
extern "C" void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of Task1_ArmGet */
  Task1_ArmGetHandle = osSemaphoreNew(1, 0, &Task1_ArmGet_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of Chassis_M3508 */
  Chassis_M3508Handle = osTimerNew(Chassis_M3508Callback, osTimerPeriodic, NULL, &Chassis_M3508_attributes);

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

  /* creation of MechaArm_Task */
  MechaArm_TaskHandle = osThreadNew(StartMechaArm_Task, NULL, &MechaArm_Task_attributes);

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

// 底盘任务
/* USER CODE END Header_StartChassisBaseTask */
void StartChassisBaseTask(void *argument)
{
  /* USER CODE BEGIN StartChassisBaseTask */
  HAL_TIM_Base_Start_IT(&htim1);
  Task1 t1 = Task1(&global_app.chassis);
  t1.Start();

  /* Infinite loop */
  for(;;)
  {
    osThreadFlagsWait(0x01u, osFlagsWaitAny, osWaitForever);
    // if (App_IsPaused()) {
    //   /* 手动接管：只跑底盘闭环，跳过任务1 FSM，避免自动逻辑和手控打架 */
    //   App_CheckVelocityTimeout(HAL_GetTick());
    //   Chassis_Update(&global_app.chassis);
    // } else {
    //   Chassis_Task1_Update(&global_app.task1);
    // }
    // osDelay(1);
    t1.Update();
  }
  /* USER CODE END StartChassisBaseTask */
}

/* USER CODE BEGIN Header_StartComms_Task */
/**
* @brief Function implementing the Comms_Task thread.
* @param argument: Not used
* @retval None
*/

// 接收命令字节的任务
/* USER CODE END Header_StartComms_Task */
void StartComms_Task(void *argument)
{
  /* USER CODE BEGIN StartComms_Task */
  uint8_t rx_byte = 0;
  /* Infinite loop */
  for(;;)
  {
    if (osMessageQueueGet(CmdQueueTask1Handle, &rx_byte, NULL, 10U) == osOK) {
      UartInteract_RxByte(&global_app.interact, rx_byte);
      while (osMessageQueueGet(CmdQueueTask1Handle, &rx_byte, NULL, 0U) == osOK) {
        UartInteract_RxByte(&global_app.interact, rx_byte);   // 把一帧的字节在本次循环凑齐
      }
    }
    App_PollEvents();
    // osDelay(1);
  }
  /* USER CODE END StartComms_Task */
}

/* USER CODE BEGIN Header_StartMechaArm_Task */
/**
* @brief Function implementing the MechaArm_Task thread.
* @param argument: Not used
* @retval None
*/

// 机械臂任务（留白）
/* USER CODE END Header_StartMechaArm_Task */
void StartMechaArm_Task(void *argument)
{
  /* USER CODE BEGIN StartMechaArm_Task */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreAcquire(Task1_ArmGetHandle, osWaitForever);
    osDelay(2000);
    arm_flag1 = 1;
  }
  /* USER CODE END StartMechaArm_Task */
}

/* Chassis_M3508Callback function */
void Chassis_M3508Callback(void *argument)
{
  /* USER CODE BEGIN Chassis_M3508Callback */
  // 任务定时器中置位，1kHz触发
  osThreadFlagsSet(ChassisMainTaskHandle, 0x01u);
  /* USER CODE END Chassis_M3508Callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

