/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    valve_driver.c
  * @brief   电磁阀驱动模块（通用 GPIO 版本）
  *          控制对象：SMC SY3120-5LZD-C6 气动电磁阀（DC24V，经外部驱动电路）
  *          控制逻辑：输出高电平 = 吸住物块（阀直线通，1-2通）
  *                   输出低电平 = 放下物块（阀L型通，2-3通）
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "valve_driver.h"

/* USER CODE BEGIN 0 */
Valve_HandleTypeDef valve_handle;
/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @brief  电磁阀驱动初始化
  * @param  hvalve : 阀句柄指针
  * @param  port   : GPIO 端口（GPIOA~GPIOK）
  * @param  pin    : 引脚号（GPIO_PIN_x）
  * @retval 0=成功, 1=参数错误（句柄为空 / 端口无效）
  * @note   初始化完成后引脚为推挽输出、初始低电平（防上电误吸）。
  *         注意：STM32 引脚最多输出 3.3V，实际驱动 24V 阀需外部驱动电路。
  */
uint8_t Valve_Driver_Init(Valve_HandleTypeDef *hvalve, GPIO_TypeDef *port, uint16_t pin)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if ((hvalve == NULL) || (port == NULL) || (pin == 0U))
  {
    return 1U;
  }

  hvalve->Port = port;
  hvalve->Pin  = pin;

  /* 使能对应 GPIO 端口的时钟 */
  if      (port == GPIOA) { __HAL_RCC_GPIOA_CLK_ENABLE(); }
  else if (port == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
  else if (port == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
  else if (port == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
  else if (port == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); }
  else if (port == GPIOF) { __HAL_RCC_GPIOF_CLK_ENABLE(); }
  else if (port == GPIOG) { __HAL_RCC_GPIOG_CLK_ENABLE(); }
  else if (port == GPIOH) { __HAL_RCC_GPIOH_CLK_ENABLE(); }
  else { return 1U; }   /* 端口无效 */

  /* 先输出低电平，避免上电瞬间误吸 */
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);

  /* 配置为推挽输出 */
  GPIO_InitStruct.Pin   = pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(port, &GPIO_InitStruct);

  return 0U;
}

/**
  * @brief  吸住物块
  *         输出高电平 → 阀直线通（1-2通），真空回路建立，吸嘴吸住物块
  * @param  hvalve : 阀句柄指针（由 Valve_Driver_Init 初始化）
  */
void Valve_Hold(Valve_HandleTypeDef *hvalve)
{
  if (hvalve == NULL) { return; }
  HAL_GPIO_WritePin(hvalve->Port, hvalve->Pin, GPIO_PIN_SET);
}

/**
  * @brief  放下物块
  *         输出低电平 → 阀L型通（2-3通），真空消失，物块放下
  * @param  hvalve : 阀句柄指针（由 Valve_Driver_Init 初始化）
  */
void Valve_Release(Valve_HandleTypeDef *hvalve)
{
  if (hvalve == NULL) { return; }
  HAL_GPIO_WritePin(hvalve->Port, hvalve->Pin, GPIO_PIN_RESET);
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
