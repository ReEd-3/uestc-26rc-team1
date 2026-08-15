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
#include "fdcan.h"
#include "stm32h723xx.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "m3508_driver.h"
#include <stdio.h>

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

double target_rpm[8] = {0}; // 创建一个数据数组
volatile uint8_t tim_flag_setcur = 0; // 定时器标志位
volatile uint8_t tim_flag_pidupdate = 0; // 定时器标志位
M3508_CAN_All m3508_can_1;
char vofa_buf[64]; // VOFA帧缓冲

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) { // 检查是否为定时器1的中断
        tim_flag_setcur = 1; // 设置定时器标志位
    }
    else if (htim->Instance ==  TIM2) {
        tim_flag_pidupdate = 1;
    }
}

// VOFA+ FireWater 遥测发送（每5个控制周期调用一次）
void VOFA_Send(void) {
    /* 上一帧 DMA 还没发完就跳过，防止覆盖正在发送的缓冲 */

    int len = sprintf(vofa_buf, "%d,%d,%d,%d,%d\n",
        (int)m3508_can_1.motors[5].position_pid.target,  // 预期位置
        (int)m3508_can_1.motors[5].position,             // 当前位置
        (int)m3508_can_1.motors[5].speed_pid.target,     // 预期速度（RPM）
        (int)m3508_can_1.motors[5].speed_pid.iir_filter.filter_status,
        (int)m3508_can_1.motors[5].current
      );               // 当前速度

    HAL_UART_Transmit_IT(&huart3, (uint8_t *)vofa_buf, len);
}

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
  MX_USART3_UART_Init();
  MX_FDCAN1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* 注意：MX_USART3_UART_Init 必须保持在 MX_FDCAN1_Init 之前！
     USART3 的 MspInit（内核时钟选择+DMA 配置）若在 FDCAN 初始化之后执行，
     会扰动已配置好的 FDCAN，导致收不到电机反馈、电机飞转。
     CubeMX 重新生成后请检查初始化顺序是否被打乱！ */
  HAL_TIM_Base_Start_IT(&htim1); // 启动定时器1中断
  HAL_TIM_Base_Start_IT(&htim2); // 启动定时器2中断
  // PID初始化和CAN初始化
  M3508_CAN_Init(&m3508_can_1, (1 << 5), &hfdcan1); // 【串口单独测试】临时注释
  M3508_SpeedPID_Init(&m3508_can_1, 4, 2, 0.02, 0.002);
  M3508_PositionPID_Init(&m3508_can_1, 3.5, 0.05, 0.02, 0.002);
  // 设置限幅
  M3508_PID_SetIntLim(&m3508_can_1.motors[5], M3508_SPEEDPID_MODE, 400);  // 速度环限幅（RPM·s）
  M3508_PID_SetIntLim(&m3508_can_1.motors[5], M3508_POSITIONPID_MODE, 500);  // 位置环限幅（计数·s）
  // 设置滤波参数
  M3508_IIRFilter_SetAlpha(&m3508_can_1.motors[5], M3508_SPEEDPID_MODE, 0.8);
  M3508_IIRFilter_SetAlpha(&m3508_can_1.motors[5], M3508_POSITIONPID_MODE, 1);

  target_rpm[5] = 0;
  // 设置目标位置（串级下内环速度目标由位置环输出给出，不再手动设置速度目标）
  // M3508_SetSpeedTarget(&m3508_can_1, target_rpm);
  M3508_SetPositionTarget(&m3508_can_1, target_rpm);
  // 设置模式
  M3508_PIDMode_Switch(&m3508_can_1.motors[5], M3508_CASCADE_MODE); // 串级模式测试
  m3508_can_1.motors[5].max_speed = 350;  // 串级内环速度限幅（RPM）

  HAL_FDCAN_Start(&hfdcan1); // 启动FDCAN模块 【串口单独测试】临时注释

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (tim_flag_pidupdate) { // 检查定时器标志位
      tim_flag_pidupdate = 0; // 清除标志位

      static uint8_t pos_init = 0;
      if (!pos_init) {  // 首拍：位置目标吸附到当前位置，避免初始误差跳变
        pos_init = 1;
        target_rpm[5] = m3508_can_1.motors[5].position;
      }
      // VOFA_Send();
      M3508_PID_Update(&m3508_can_1); // 【串口单独测试】临时注释
      
    }
    if (tim_flag_setcur) {
      tim_flag_setcur = 0;
      target_rpm[5] += 20;  // 位置目标每拍递增 45 计数（≈330 RPM 当量）
      if (target_rpm[5] > M3508_ENCODER_RESOLUTION) {
        target_rpm[5] -= M3508_ENCODER_RESOLUTION;
      }
      M3508_SetPositionTarget(&m3508_can_1, target_rpm);
      M3508_CAN_CurrentUpdate(&m3508_can_1);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 34;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
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
