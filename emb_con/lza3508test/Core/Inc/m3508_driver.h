#ifndef M3508_DRIVER_H
#define M3508_DRIVER_H

#define M3508_CAN_ID_BASE 0x200  // M3508电机的CAN ID基地址
#define M3508_CURRENT_MAX 16384  // M3508电机的最大电流值
#define M3508_CURRENT_MIN -16384  // 反向电流值
#define M3508_ENCODER_RESOLUTION 8192  // 编码器分辨率
#define M3508_GEAR_RATIO 3591 / 187  // M3508电机减速比

enum M3508_Status {
    M3508_OFF = 0,
    M3508_ON = 1
};

typedef struct {
    FDCAN_HandleTypeDef *hfdcan;  // FDCAN句柄
    uint8_t status;  // 电机状态
    uint8_t can_id;  // 电机的CAN ID
    int16_t current;  // 电机电流值
    int16_t speed;    // 电机速度值
    uint16_t position; // 电机位置值
    int8_t temperature; // 电机温度值
} M3508_HandleTypeDef;

typedef struct {
    FDCAN_HandleTypeDef *hfdcan;  // FDCAN句柄
    M3508_HandleTypeDef motors[8];  // 四个M3508电机的句柄数组
} M3508_CAN_ALL;

HAL_StatusTypeDef M3508_Init(M3508_HandleTypeDef *motor, FDCAN_HandleTypeDef *hfdcan, uint8_t can_id);
HAL_StatusTypeDef M3508_SetCurrent(M3508_HandleTypeDef *motor, int16_t current);
HAL_StatusTypeDef M3508_CAN_Init(M3508_HandleTypeDef *motor, uint8_t motor_ids, FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef M3508_ReadStatus(M3508_CAN_ALL *m3508_can);
#endif