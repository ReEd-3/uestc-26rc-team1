#include "fdcan_std.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_fdcan.h"

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
    sFilterConfig.FilterID1 = 0x000;  // 设置滤波器ID1为0x000，表示接收所有报文
    sFilterConfig.FilterID2 = 0x000; // 设置滤波器ID2为0x000，表示接收所有报文

    return HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig);
}



