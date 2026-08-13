#ifndef M3508_DRIVER_H
#define M3508_DRIVER_H

#include "pid.h"

#define M3508_CAN_ID_BASE 0x200  // M3508电机的CAN ID基地址
#define M3508_CURRENT_MAX 16384  // M3508电机的最大电流值
#define M3508_CURRENT_MIN -16384  // 反向电流值
#define M3508_ENCODER_RESOLUTION 8192  // 编码器分辨率
#define M3508_GEAR_RATIO (3591.0f / 187.0f)  // M3508电机减速比
#define M3508_CONTROL_ID_LOW 0x200  // 1-4电机的控制帧id
#define M3508_CONTROL_ID_HIGH 0x1FF  // 5-8电机的控制帧id

typedef enum {
    M3508_OFF = 0,
    M3508_ON = 1
} M3508_Status;

// low指的是1-4号电机，high是5-8号
typedef enum {
    M3508_GROUP_LOW = 0,
    M3508_GROUP_HIGH = 1
} M3508_Motor_Group;

typedef struct {
    FDCAN_HandleTypeDef *hfdcan;  // FDCAN句柄
    uint8_t status;  // 电机状态
    uint8_t can_id;  // 电机的CAN ID
    int16_t current;  // 电机电流值
    int16_t speed;    // 电机速度值
    uint16_t position;  // 电机位置值
    int8_t temperature;  // 电机温度值
    PID_t speed_pid;  // 速度环 
    PID_t position_pid;  // 位置环
} M3508_HandleTypeDef;

typedef struct {
    FDCAN_HandleTypeDef *hfdcan;  // FDCAN句柄
    M3508_HandleTypeDef motors[8];  // 8个M3508电机的句柄数组
} M3508_CAN_All;

HAL_StatusTypeDef M3508_Init(M3508_HandleTypeDef *motor, FDCAN_HandleTypeDef *hfdcan, uint8_t can_id);
HAL_StatusTypeDef M3508_SetCurrent(M3508_CAN_All *m3508_can, M3508_Motor_Group group_id, int16_t *current);
HAL_StatusTypeDef M3508_CAN_Init(M3508_CAN_All *m3508_can, uint8_t motor_ids, FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef M3508_ReadStatus(M3508_CAN_All *m3508_can);

// === 速度环 PID 控制 ===
void M3508_SetSpeedTarget(M3508_CAN_All *m3508_can, double *target_rpm);
void M3508_SpeedPID_MotorInit(M3508_HandleTypeDef *motor, double Kp, double Ki, double Kd, double dt);
void M3508_SpeedPID_Init(M3508_CAN_All *m3508_can, double Kp, double Ki, double Kd, double dt);
void M3508_SpeedPID_Update(M3508_CAN_All *m3508_can);  // 每个控制周期调用：读取→PID计算→CAN发送

// === 位置环 PID 控制 ===
void M3508_SetPositionTarget(M3508_CAN_All *m3508_can, double *target_rpm);
void M3508_PositionPID_MotorInit(M3508_HandleTypeDef *motor, double Kp, double Ki, double Kd, double dt);
void M3508_PositionPID_Init(M3508_CAN_All *m3508_can, double Kp, double Ki, double Kd, double dt);
void M3508_PositionPID_Update(M3508_CAN_All *m3508_can);  // 每个控制周期调用：读取→PID计算→CAN发送

#endif