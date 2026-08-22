#ifndef YIS506_H
#define YIS506_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C"
#endif

#define YIS506_EULER_ID   0x0CF02959UL
#define YIS506_GYRO_ID    0x0CF02A59UL
#define YIS506_QUAT_ID    0x0CF03059UL

#define YIS506_EULER_BUF  3u
#define YIS506_GYRO_BUF   4u
#define YIS506_QUAT_BUF   5u

typedef union {
    uint32_t udata;
    float fdata;
} Yis_Data;

typedef struct Y5 {
    FDCAN_HandleTypeDef *hfdcan;
    Yis_Data accel[3];
    uint8_t gyro[8];
    Yis_Data euler[3];  // Pitch, Roll, Yaw
    Yis_Data quat[4];  // w, x, y, z
    double yaw;
    uint8_t initialized;
} Yis506;

void Yis506_Init(Yis506 *yis, FDCAN_HandleTypeDef *hfdcan);
void Yis506_Update(Yis506 *yis);
double Yis506_GetYawRad(Yis506 *yis);

#ifdef __cplusplus
extern "C"
#endif

#endif