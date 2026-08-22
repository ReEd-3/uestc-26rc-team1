#include "yis506.h"
#include <math.h>

#define YIS506_EULER_SCALE   0.0078125f
#define YIS506_EULER_OFFSET  250.0f
#define YIS506_PI            3.14159265358979323846

static float Yis506_EulerToRad(uint16_t raw)
{
    float deg = raw * YIS506_EULER_SCALE - YIS506_EULER_OFFSET;
    return deg * YIS506_PI / 180.0f;
}

void Yis506_Init(Yis506 *yis, FDCAN_HandleTypeDef *hfdcan)
{
    if (yis == NULL || hfdcan == NULL) {
        return;
    }

    yis->hfdcan = hfdcan;
    yis->initialized = 0;

    FDCAN_FilterTypeDef filter = {0};
    filter.IdType       = FDCAN_EXTENDED_ID;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXBUFFER;
    filter.FilterID2    = 0x1FFFFFFF;   // 精确匹配 29 位扩展帧

    // 欧拉角
    filter.FilterIndex   = YIS506_EULER_BUF;
    filter.RxBufferIndex = YIS506_EULER_BUF;
    filter.FilterID1     = YIS506_EULER_ID;
    HAL_FDCAN_ConfigFilter(hfdcan, &filter);

    // 角速度（可选）
    filter.FilterIndex   = YIS506_GYRO_BUF;
    filter.RxBufferIndex = YIS506_GYRO_BUF;
    filter.FilterID1     = YIS506_GYRO_ID;
    HAL_FDCAN_ConfigFilter(hfdcan, &filter);

    // 四元数（可选）
    filter.FilterIndex   = YIS506_QUAT_BUF;
    filter.RxBufferIndex = YIS506_QUAT_BUF;
    filter.FilterID1     = YIS506_QUAT_ID;
    HAL_FDCAN_ConfigFilter(hfdcan, &filter);

    HAL_FDCAN_Start(hfdcan);
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_BUFFER_NEW_MESSAGE, 0);
}

void Yis506_Update(Yis506 *yis)
{
    if (yis == NULL || yis->hfdcan == NULL) {
        return;
    }

    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];

    // 欧拉角
    if (HAL_FDCAN_GetRxMessage(yis->hfdcan, YIS506_EULER_BUF, &header, data) == HAL_OK) {
        uint16_t pitch_raw = data[0] | (data[1] << 8);
        uint16_t roll_raw  = data[2] | (data[3] << 8);
        uint16_t yaw_raw   = data[4] | (data[5] << 8);

        yis->euler[0].fdata = Yis506_EulerToRad(pitch_raw);
        yis->euler[1].fdata = Yis506_EulerToRad(roll_raw);
        yis->euler[2].fdata = Yis506_EulerToRad(yaw_raw);

        yis->initialized = 1;
    }
    yis->yaw = Yis506_GetYawRad(yis);
    // 角速度 / 四元数按需解析
}

double Yis506_GetYawRad(Yis506 *yis)
{
    if (yis == NULL) {
        return 0.0;
    }
    return yis->euler[2].fdata;
}