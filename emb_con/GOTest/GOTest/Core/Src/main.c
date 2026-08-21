/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "GO8010_driver.h"
#include <stdio.h>
#include <string.h>
#include "gom_protocol.h"
#include "pid.h"   
#include "go8010_pos_ctrl.h"
#include "box_flip_task.h"
#include "valve_driver.h"

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

/* USER CODE BEGIN PV */
	GO8010_Motor_t motor;
	GO8010_PosCtrl_t ctrl;
	char send_buf[128];
//	//=====翻转状态机新增=====
//	
	extern Valve_HandleTypeDef valve_handle;  //你的电磁阀句柄，根据你实际名字修改
	BoxFlipTask flip_task;

//	uint8_t go_180_flag = 1;
//	uint32_t tick_timer = 0;
//	
//	#define TEST_ARRIVE_FUNC
//	#ifdef TEST_ARRIVE_FUNC
//	static uint8_t test_stage = 0;
//	static uint32_t test_tick = 0;
//	#endif
//	char send_buf[128];

osThreadId_t FlipBoxTaskHandle;//任务句柄，保存任务创建成功之后返回的ID

//任务属性结构体：描述任务需要什么配置
const osThreadAttr_t FlipBoxTask_attributes = {
  .name = "FlipBoxTask",   //调试名字，RTOS‑RTOS2调试器看的，字符串，不影响运行
  .stack_size = 512 * 4,   //栈大小，512字，翻转任务够用
  .priority = (osPriority_t) osPriorityHigh,//任务优先级
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void StartFlipBoxTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_UART9_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
	GO8010_Motor_Init(&motor, &huart9, GPIOF, GPIO_PIN_14, 1);
	GO8010_PosCtrl_Init(&ctrl);
	
	//========翻转任务初始化========
	//绑定：位置控制器、电机、电磁阀句柄
	
	valve_handle.Port = GPIOF;
	valve_handle.Pin  = GPIO_PIN_6;
	BoxFlipTask_Init(&flip_task, &ctrl, &motor, &valve_handle);
//	//可选：修改吸附、释放延时，不调用就用默认250/200
//	BoxFlipTask_SetDelay(&flip_task, 5000, 2000);
	
	
	FlipBoxTaskHandle = osThreadNew(StartFlipBoxTask, NULL, &FlipBoxTask_attributes);//创建FreeRTOS翻转任务（只是注册，还没开始跑）

//	HAL_Delay(1000);
	//可以加调试判断：如果返回NULL，代表堆不足，任务创建失败
	if(FlipBoxTaskHandle == NULL)
	{
	  //任务创建失败！堆内存不够，需要在CubeMX调大configTOTAL_HEAP_SIZE
	}
	
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	uint32_t tick = HAL_GetTick();

//	//【1】电机位置闭环，必须持续循环调用！刷新real_pos_out，到位判断才生效
//	GO8010_PosCtrl_Run(&ctrl, &motor);
//	
//	static uint32_t last_print_tick = 0;
//    if(HAL_GetTick() - last_print_tick >= 200)
//    {
//        last_print_tick = HAL_GetTick();
//        uint8_t arrive = GO8010_PosCtrl_CheckArrived(&ctrl);
//        snprintf(send_buf, sizeof(send_buf),
//            "tar:%.3f | real:%.3f | arrive:%d\r\n",
//            ctrl.pos_target_out, ctrl.real_pos_out, arrive);
//        HAL_UART_Transmit(&huart1, (uint8_t*)send_buf, strlen(send_buf), 20);
//    }
//	//【2】运行翻转状态机引擎
//	BoxFlipTask_Update(&flip_task, tick);
//    BoxFlipHostEvent evt = BoxFlipTask_PopHostEvent(&flip_task);
//	
//	static uint8_t task_once = 0;
//	if(!task_once)
//	{
//		BoxFlipTask_Start(&flip_task);
//		task_once = 1;
//	}
//	
//	if(evt != BOX_FLIP_EVENT_NONE)
//	{
//		if(evt == BOX_FLIP_EVENT_TASK_FINISH)
//		{
//			//整套翻转完成，可以打印日志
//		}
//	}
}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief 翻转箱子RTOS任务外壳，周期5ms
 */
void StartFlipBoxTask(void *argument)//RTOS任务包装
{
   (void)argument;
  static uint8_t first_run = 1;
  uint32_t tick;
  BoxFlipHostEvent evt;

  for(;;)
  {
    if(first_run)
    {
        BoxFlipTask_Start(&flip_task);
        first_run = 0;
    }

    tick = HAL_GetTick();
    GO8010_PosCtrl_Run(&ctrl, &motor);
    BoxFlipTask_Update(&flip_task, tick);

    //取出事件
    evt = BoxFlipTask_PopHostEvent(&flip_task);
    //如果检测到整套任务完成，延时一小段，再次启动翻转
    if(evt == BOX_FLIP_EVENT_TASK_FINISH)
    {
        osDelay(1000);   //间隔1秒后再次执行，可修改
        BoxFlipTask_Start(&flip_task);
    }

    osDelay(5);
  }
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
