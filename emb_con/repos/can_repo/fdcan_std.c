#include "stm32h7xx_hal.h"
#include "fdcan_std.h"
#include "stm32h7xx_hal_fdcan.h"

/**
  * @brief  配置默认滤波器：掩码模式，接受所有标准ID报文，存入RXFIFO0
  * @param  hfdcan  FDCAN句柄指针（&hfdcan1 / &hfdcan2 / &hfdcan3）
  * @retval HAL_OK     滤波器配置成功
  *         HAL_ERROR  滤波器槽位未分配（Init.StdFiltersNbr == 0），或HAL配置失败
  */
HAL_StatusTypeDef HAL_FDCAN_StdDefault_ConfigFilter(FDCAN_HandleTypeDef *hfdcan) {
    /* 滤波器槽位没分配，直接报错 */
    if (hfdcan->Init.StdFiltersNbr == 0) {
        return HAL_ERROR;
    }

    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;  // 设置滤波器类型为掩码模式
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;  // 设置滤波器配置为接收FIFO0
    // 默认接收所有消息
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x000;

    return HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig);
}

/**
  * @brief  初始化发送报文头——填入固定格式字段，应用层只需关心ID和数据长度
  * @param  header      指向 TxHeader 结构体的指针
  * @param  identifier  CAN报文ID（标准帧: 0x000~0x7FF，扩展帧: 0x00000000~0x1FFFFFFF）
  * @param  data_length 数据长度（经典CAN: FDCAN_DLC_BYTES_0 ~ FDCAN_DLC_BYTES_8）
  * @param  hfdcan      FDCAN句柄指针，用于继承 FrameFormat 配置
  * @retval HAL_OK      初始化成功
  */
HAL_StatusTypeDef HAL_FDCAN_StdDefault_TxHeaderInit(FDCAN_TxHeaderTypeDef *header, uint32_t identifier, uint8_t data_length, FDCAN_HandleTypeDef *hfdcan) {
    header->Identifier = identifier; // 设置报文ID
    header->IdType = FDCAN_STANDARD_ID; // 设置ID类型为标准
    header->TxFrameType = FDCAN_DATA_FRAME; // 设置帧类型为数据帧
    header->DataLength = data_length; // 设置数据长度
    header->ErrorStateIndicator = FDCAN_ESI_ACTIVE;  // 设置错误状态指示器为活动
    header->BitRateSwitch = FDCAN_BRS_OFF;  // 设置比特率切换为关闭
    header->FDFormat = hfdcan->Init.FrameFormat;  // 设置FD格式为与FDCAN实例一致
    header->TxEventFifoControl = FDCAN_NO_TX_EVENTS;  // 设置TX事件FIFO控制为无TX事件
    header->MessageMarker = 0;

    return HAL_OK;
}

/**
  * @brief  初始化接收报文头——与发送头对称，少两个发送专属字段
  * @param  header      指向 RxHeader 结构体的指针
  * @param  identifier  期望的CAN报文ID（实际接收后会被HAL覆写）
  * @param  data_length 期望的数据长度（实际接收后会被HAL覆写）
  * @param  hfdcan      FDCAN句柄指针
  * @retval HAL_OK      初始化成功
  */
HAL_StatusTypeDef HAL_FDCAN_StdDefault_RxHeaderInit(FDCAN_RxHeaderTypeDef *header, uint32_t identifier, uint8_t data_length, FDCAN_HandleTypeDef *hfdcan) {
    header->Identifier = identifier; // 设置报文ID
    header->IdType = FDCAN_STANDARD_ID; // 设置ID类型为标准
    header->RxFrameType = FDCAN_DATA_FRAME; // 设置帧类型为数据帧
    header->DataLength = data_length; // 设置数据长度
    header->ErrorStateIndicator = FDCAN_ESI_ACTIVE;  // 设置错误状态指示器为活动
    header->BitRateSwitch = FDCAN_BRS_OFF;  // 设置比特率切换为关闭
    header->FDFormat = hfdcan->Init.FrameFormat;  // 设置FD格式为与FDCAN实例一致

    return HAL_OK;
}

/**
  * @brief  发送CAN报文——将报文头和8字节数据写入TXFIFO队列
  * @param  header  发送报文头指针（TxHeader）
  * @param  hfdcan  FDCAN句柄指针
  * @param  data    待发送的8字节数据缓冲区
  * @retval HAL_OK      报文已写入TXFIFO，等待发送
  *         HAL_ERROR  TXFIFO已满或无可用槽位
  * @note   内部直接调用 HAL_FDCAN_AddMessageToTxFifoQ
  */
HAL_StatusTypeDef HAL_FDCAN_Std_SendMessage(FDCAN_RxHeaderTypeDef *header, FDCAN_HandleTypeDef *hfdcan, uint8_t *data) {
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, (FDCAN_TxHeaderTypeDef *)header, data);
}

/**
  * @brief  接收CAN报文——先检查RXFIFO是否有数据，有则读取
  * @param  header  接收报文头指针（RxHeader，由HAL填入实际ID/长度等信息）
  * @param  hfdcan  FDCAN句柄指针
  * @param  fifo_id 接收FIFO编号（FDCAN_RX_FIFO0 或 FDCAN_RX_FIFO1）
  * @param  data    存放接收到数据的8字节缓冲区
  * @retval HAL_OK      成功读取一帧报文
  *         HAL_ERROR  RXFIFO为空，当前无数据可读
  * @note   先检查FIFO水位再读取，避免空FIFO时调用GetRxMessage导致阻塞
  */
HAL_StatusTypeDef HAL_FDCAN_Std_ReceiveMessage(FDCAN_RxHeaderTypeDef *header, FDCAN_HandleTypeDef *hfdcan, uint32_t fifo_id, uint8_t *data) {
    if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, fifo_id) > 0) {
        return HAL_FDCAN_GetRxMessage(hfdcan, fifo_id, header, data);
    }
    else {
        return HAL_ERROR; // 没有消息可接收
    }
}



