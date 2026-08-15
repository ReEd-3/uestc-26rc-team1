#ifndef FDCAN_STD_H
#define FDCAN_STD_H

#include "stm32h7xx_hal.h"

HAL_StatusTypeDef HAL_FDCAN_StdDefault_ConfigFilter(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_StdDefault_TxHeaderInit(FDCAN_TxHeaderTypeDef *header, uint32_t identifier, uint8_t data_length, FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_StdDefault_RxHeaderInit(FDCAN_RxHeaderTypeDef *header, uint32_t identifier, uint8_t data_length, FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_Std_SendMessage(FDCAN_TxHeaderTypeDef *header, FDCAN_HandleTypeDef *hfdcan, uint8_t *data);
HAL_StatusTypeDef HAL_FDCAN_Std_ReceiveMessage(FDCAN_RxHeaderTypeDef *header, FDCAN_HandleTypeDef *hfdcan, uint32_t fifo_id, uint8_t *data);

#endif /* FDCAN_STD_H */



