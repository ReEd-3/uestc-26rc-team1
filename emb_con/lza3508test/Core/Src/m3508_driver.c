#include "stm32h7xx_hal.h"
#include "fdcan_std.h"
#include "m3508_driver.h"
#include "pid.h"
#include <stdint.h>

HAL_StatusTypeDef M3508_Init(M3508_HandleTypeDef *motor, FDCAN_HandleTypeDef *hfdcan, uint8_t can_id) {
    if (motor == NULL || hfdcan == NULL) {
        return HAL_ERROR; // 检查指针是否为空
    }

    motor->status = M3508_OFF; // 初始化电机状态为关闭
    motor->hfdcan = hfdcan;
    motor->can_id = can_id;
    motor->current = 0;
    motor->speed = 0;
    motor->position = 0;
    motor->temperature = 0;

    return HAL_OK; // 初始化成功
}

HAL_StatusTypeDef M3508_CAN_Init(M3508_CAN_ALL *m3508_can, uint8_t motor_ids, FDCAN_HandleTypeDef *hfdcan) {
    // 检测缓冲和滤波器数量
    if (hfdcan->Init.StdFiltersNbr < 8 ||
        hfdcan->Init.RxBuffersNbr  < 8) {
        return HAL_ERROR;
    }

    m3508_can->hfdcan = hfdcan;

    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType       = FDCAN_STANDARD_ID;
    sFilterConfig.FilterType   = FDCAN_FILTER_MASK;      // 掩码模式：精确匹配
    sFilterConfig.FilterID2    = 0x7FF;                   // 全掩码：所有位必须匹配
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXBUFFER; // 存入专用 RxBuffer
    
    for (int i = 0; i < 8; i++) {
        if (M3508_Init(&m3508_can->motors[i], hfdcan, i + 1) != HAL_OK) {
            return HAL_ERROR; // 初始化失败
        }
        if (motor_ids & (1 << i)) { // 检查每一位是否为1
            m3508_can->motors[i].status = M3508_ON;
            sFilterConfig.FilterIndex   = i;              // Filter[i]
            sFilterConfig.RxBufferIndex = i;              // → RxBuffer[i]
            sFilterConfig.FilterID1     = M3508_CAN_ID_BASE + i;    // 只匹配 0x200+i

            if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
            {
                /* 实际使用中可在此处记录第一个失败的位置 */
                return HAL_ERROR;
            }
        }
    }

    /* 全局滤波器：拒绝所有未被上面 Filter 匹配的帧 */
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
            FDCAN_REJECT,        // 非匹配标准帧 → 拒收
            FDCAN_REJECT,        // 非匹配扩展帧 → 拒收
            FDCAN_REJECT_REMOTE, // 拒绝标准远程帧
            FDCAN_REJECT_REMOTE) // 拒绝扩展远程帧
        != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK; // 初始化成功
}

HAL_StatusTypeDef M3508_SetCurrent(M3508_HandleTypeDef *motor, int16_t current) {

    if (current < M3508_CURRENT_MIN) {
        current = M3508_CURRENT_MIN; // 限制最小电流值
    } 
    else if (current > M3508_CURRENT_MAX) {
        current = M3508_CURRENT_MAX; // 限制最大电流值
    }

    motor->current = current;

    uint8_t data[8] = {0}; // 创建一个8字节的数据数组
    FDCAN_TxHeaderTypeDef CAN_TxHeader; // 创建发送报文头

    if (motor->can_id <= 4) {
        data[2 * (motor->can_id - 1)] = (current >> 8) & 0xFF; // 高字节
        data[2 * motor->can_id - 1] = current & 0xFF; // 低字节
        HAL_FDCAN_StdDefault_TxHeaderInit(&CAN_TxHeader, 0x200, 8, motor->hfdcan); // 初始化发送报文头
        return (HAL_FDCAN_Std_SendMessage(&CAN_TxHeader, motor->hfdcan, data)); // 发送FDCAN报文
    }
    else {
        data[2 * motor->can_id - 10] = (current >> 8) & 0xFF; // 高字节
        data[2 * motor->can_id - 9] = current & 0xFF; // 低字节
        HAL_FDCAN_StdDefault_TxHeaderInit(&CAN_TxHeader, 0x1FF, 8, motor->hfdcan); // 初始化发送报文头
        return (HAL_FDCAN_Std_SendMessage(&CAN_TxHeader, motor->hfdcan, data)); // 发送FDCAN报文
    }
}

// 读取总线上所有电机的状态
HAL_StatusTypeDef M3508_ReadStatus(M3508_CAN_ALL *m3508_can)
{
    HAL_StatusTypeDef result = HAL_ERROR;

    for (uint8_t i = 0; i < 8; i++)
    {
        /* 跳过未使能的电机 */
        if (m3508_can->motors[i].status != M3508_ON)
            continue;

        uint8_t buf_idx = i;  // can_id=1 → RxBuffer[0], can_id=2 → RxBuffer[1], ...

        /* 检查该电机专用缓冲区是否有新帧（硬件保证是该 ID 的帧） */
        if (HAL_FDCAN_IsRxBufferMessageAvailable(m3508_can->hfdcan, buf_idx) == 0)
            continue;

        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t data[8];

        if (HAL_FDCAN_GetRxMessage(m3508_can->hfdcan, buf_idx, &RxHeader, data) != HAL_OK)
            continue;

        /* 硬件已通过滤波器 ID 匹配，无需软件再比对 Identifier */
        /* M3508 反馈帧格式 (8 bytes):
           [0..1] = 机械角度 (uint16, 0~8191)
           [2..3] = 转速 (int16, RPM)
           [4..5] = 转矩电流 (int16)
           [6]    = 温度 (°C)
           [7]    = 保留 */
        m3508_can->motors[i].position    =  (uint16_t)((data[0] << 8) | data[1]);
        m3508_can->motors[i].speed       =  (int16_t)((data[2] << 8) | data[3]);
        m3508_can->motors[i].current     =  (int16_t)((data[4] << 8) | data[5]);
        m3508_can->motors[i].temperature =  (int8_t)data[6];

        result = HAL_OK;
    }

    return result;
}
