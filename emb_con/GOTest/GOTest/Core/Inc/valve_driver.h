/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    valve_driver.h
  * @brief   电磁阀驱动模块头文件（通用 GPIO 版本）
  *          控制对象：SMC SY3120-5LZD-C6 气动电磁阀（DC24V，经外部驱动电路）
  *          控制逻辑：输出高电平 = 吸住物块（阀直线通，1-2通）
  *                   输出低电平 = 放下物块（阀L型通，2-3通）
  *
  * 用法示例：
  *   Valve_HandleTypeDef hvalve;
  *   Valve_Driver_Init(&hvalve, GPIOF, GPIO_PIN_6);  // 传入端口和引脚
  *   Valve_Hold(&hvalve);        // 吸住
  *   Valve_Release(&hvalve);     // 放下
  *   多个阀：每个阀一个 Valve_HandleTypeDef，分别 Init/Hold/Release
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __VALVE_DRIVER_H
#define __VALVE_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* 阀对象句柄：记录该阀使用的 GPIO 端口和引脚 */
typedef struct
{
  GPIO_TypeDef *Port;   /* GPIO 端口，如 GPIOF */
  uint16_t      Pin;    /* 引脚号，如 GPIO_PIN_6 */
} Valve_HandleTypeDef;

/* 对外接口 */
/**
  * @brief  电磁阀驱动初始化
  * @param  hvalve : 阀句柄指针
  * @param  port   : GPIO 端口（GPIOA~GPIOH，STM32H723 无 GPIOI~K）
  * @param  pin    : 引脚号（GPIO_PIN_x）
  * @retval 0=成功, 1=参数错误
  */
uint8_t Valve_Driver_Init(Valve_HandleTypeDef *hvalve, GPIO_TypeDef *port, uint16_t pin);

/** @brief 吸住物块：输出高电平 */
void Valve_Hold(Valve_HandleTypeDef *hvalve);

/** @brief 放下物块：输出低电平 */
void Valve_Release(Valve_HandleTypeDef *hvalve);

#ifdef __cplusplus
}
#endif

#endif /* __VALVE_DRIVER_H */
