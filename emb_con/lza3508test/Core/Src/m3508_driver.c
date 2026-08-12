#include "stm32h7xx_hal.h"
#include "fdcan_std.h"
#include "m3508_driver.h"
#include "pid.h"

// 对总线上单个电机进行初始化
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

// 对控制电机的总线进行初始化
HAL_StatusTypeDef M3508_CAN_Init(M3508_CAN_All *m3508_can, uint8_t motor_ids, FDCAN_HandleTypeDef *hfdcan) {
    // 检测缓冲和滤波器数量
    if (hfdcan->Init.StdFiltersNbr < 8 ||
        hfdcan->Init.RxBuffersNbr  < 8) {
        return HAL_ERROR;
    }

    // 指定使用的can句柄
    m3508_can->hfdcan = hfdcan;

    // 指定滤波器配置
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType       = FDCAN_STANDARD_ID;
    sFilterConfig.FilterType   = FDCAN_FILTER_MASK;      // 掩码模式：精确匹配
    sFilterConfig.FilterID2    = 0x7FF;                   // 全掩码：所有位必须匹配
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXBUFFER; // 存入专用 RxBuffer
    
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

    for (int i = 0; i < 8; i++) {

        // 初始化每个电机，主要是id
        if (M3508_Init(&m3508_can->motors[i], hfdcan, i + 1) != HAL_OK) {
            return HAL_ERROR; // 初始化失败
        }

        // 检查是否启用，并配置滤波器
        if (motor_ids & (1 << i)) { // 检查每一位是否为1
            m3508_can->motors[i].status = M3508_ON;
            sFilterConfig.FilterIndex   = i;              // Filter[i]
            sFilterConfig.RxBufferIndex = i;              // → RxBuffer[i]
            sFilterConfig.FilterID1     = M3508_CAN_ID_BASE + i + 1;  // 电机 ID=i+1 → 反馈 ID=0x200+(i+1)

            if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
            {
                /* 实际使用中可在此处记录第一个失败的位置 */
                return HAL_ERROR;
            }
        }
    }

    return HAL_OK; // 初始化成功
}

// 设置一整个组的电机电流
HAL_StatusTypeDef M3508_SetCurrent(M3508_CAN_All *m3508_can, M3508_Motor_Group group_id, int16_t *current) {

    uint8_t data[8] = {0}; // 创建一个8字节的数据数组
    FDCAN_TxHeaderTypeDef CAN_TxHeader; // 创建发送报文头

    for (int i = 0; i < 4; i++){
        if ((group_id == M3508_GROUP_LOW && m3508_can->motors[i].status == M3508_OFF)
        || (group_id == M3508_GROUP_HIGH && m3508_can->motors[i + 4].status == M3508_OFF)) {
            continue;
        }
        if (current[i] < M3508_CURRENT_MIN) {
        current[i] = M3508_CURRENT_MIN; // 限制反向最大电流值
        } 
        else if (current[i] > M3508_CURRENT_MAX) {
            current[i] = M3508_CURRENT_MAX; // 限制最大电流值
        }
        data[2 * i] = (current[i] >> 8) & 0xFF;  // 高字节
        data[2 * i + 1] = current[i] & 0xFF;
    }

    if (group_id == M3508_GROUP_LOW) {
        HAL_FDCAN_StdDefault_TxHeaderInit(&CAN_TxHeader, M3508_CONTROL_ID_LOW, 8, m3508_can->hfdcan); // 初始化发送报文头
        return (HAL_FDCAN_Std_SendMessage(&CAN_TxHeader, m3508_can->hfdcan, data)); // 发送FDCAN报文
    }
    else if (group_id == M3508_GROUP_HIGH) {
        HAL_FDCAN_StdDefault_TxHeaderInit(&CAN_TxHeader, M3508_CONTROL_ID_HIGH, 8, m3508_can->hfdcan); // 初始化发送报文头
        return (HAL_FDCAN_Std_SendMessage(&CAN_TxHeader, m3508_can->hfdcan, data)); // 发送FDCAN报文
    }
    else {
        return HAL_ERROR;
    }
}

// 读取总线上所有电机的状态
HAL_StatusTypeDef M3508_ReadStatus(M3508_CAN_All *m3508_can)
{
    HAL_StatusTypeDef result = HAL_ERROR;

    for (uint8_t i = 0; i < 8; i++)
    {
        /* 跳过未使能的电机 */
        if (m3508_can->motors[i].status != M3508_ON) {
            continue;
        }

        uint8_t buf_idx = i;  // can_id=1 → RxBuffer[0], can_id=2 → RxBuffer[1], ...

        /* 检查该电机专用缓冲区是否有新帧（硬件保证是该 ID 的帧） */
        if (HAL_FDCAN_IsRxBufferMessageAvailable(m3508_can->hfdcan, buf_idx) == 0) {
            continue; // 没有新帧，跳过
        }

        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t data[8];

        if (HAL_FDCAN_GetRxMessage(m3508_can->hfdcan, buf_idx, &RxHeader, data) != HAL_OK) {
            continue; // 获取消息失败，跳过
        }

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

// ============================================================
//  速度环 PID 控制
// ============================================================

void M3508_SetSpeedTarget(M3508_CAN_All *m3508_can, double *target_rpm) {
    for(int i = 0; i < 8; i++) {
        if (m3508_can->motors[i].status == M3508_OFF) {
            continue;
        }
        m3508_can->motors[i].speed_pid.target = target_rpm[i];
    }
}

void M3508_SpeedPID_MotorInit(M3508_HandleTypeDef *motor, double Kp, double Ki, double Kd, double dt) {
    PID_Init(&motor->speed_pid, Kp, Ki, Kd, dt);
}

void M3508_SpeedPID_Init(M3508_CAN_All *m3508_can, double Kp, double Ki, double Kd, double dt) {
    for(int i = 0; i < 8; i++) {
        if (m3508_can->motors[i].status == M3508_OFF) {
            continue;
        }
        M3508_SpeedPID_MotorInit(&m3508_can->motors[i], Kp, Ki, Kd, dt);  // dt单位s
    }
}

void M3508_SpeedPID_Update(M3508_CAN_All *m3508_can) {

    /* 读取所有电机反馈 */
    M3508_ReadStatus(m3508_can);

    int16_t cur_low[4]  = {0};
    int16_t cur_high[4] = {0};

    /* 对每个电机进行 PID 计算 */
    for (int i = 0; i < 8; i++) {

        if (m3508_can->motors[i].status != M3508_ON) {
            continue;  // 未启用的电机电流保持 0
        }

        M3508_HandleTypeDef *motor = &(m3508_can->motors[i]);

        /* 将反馈转速写入 PID，计算控制量 */
        motor->speed_pid.current = motor->speed;
        double output = PID_Compute(&motor->speed_pid);

        /* 限幅到电流范围 [-16384, 16384] */
        if (output > M3508_CURRENT_MAX)  output = M3508_CURRENT_MAX;
        if (output < M3508_CURRENT_MIN)  output = M3508_CURRENT_MIN;

        /* 分入 LOW 组 (0~3) 或 HIGH 组 (4~7) */
        if (i < 4) {
            cur_low[i] = (int16_t)output;
        } else {
            cur_high[i - 4] = (int16_t)output;
        }
    }

    /* CAN 发送：一个 ID 控制一组 4 个电机 */
    M3508_SetCurrent(m3508_can, M3508_GROUP_LOW,  cur_low);
    M3508_SetCurrent(m3508_can, M3508_GROUP_HIGH, cur_high);
}

// ============================================================
//  位置环 PID 控制
// ============================================================

void M3508_SetPositionTarget(M3508_CAN_All *m3508_can, double *target_rpm) {
    for(int i = 0; i < 8; i++) {
        if (m3508_can->motors[i].status == M3508_OFF) {
            continue;
        }
        m3508_can->motors[i].position_pid.target = target_rpm[i];
    }
}

void M3508_PositionPID_MotorInit(M3508_HandleTypeDef *motor, double Kp, double Ki, double Kd, double dt) {
    PID_Init(&motor->position_pid, Kp, Ki, Kd, dt);
}

void M3508_PositionPID_Init(M3508_CAN_All *m3508_can, double Kp, double Ki, double Kd, double dt) {
    for(int i = 0; i < 8; i++) {
        if (m3508_can->motors[i].status == M3508_OFF) {
            continue;
        }
        M3508_PositionPID_MotorInit(&m3508_can->motors[i], Kp, Ki, Kd, dt);  // dt单位s
    }
}

void M3508_PositionPID_Update(M3508_CAN_All *m3508_can) {

    /* 读取所有电机反馈 */
    M3508_ReadStatus(m3508_can);

    int16_t cur_low[4]  = {0};
    int16_t cur_high[4] = {0};

    /* 对每个电机进行 PID 计算 */
    for (int i = 0; i < 8; i++) {

        if (m3508_can->motors[i].status != M3508_ON) {
            continue;  // 未启用的电机电流保持 0
        }

        M3508_HandleTypeDef *motor = &(m3508_can->motors[i]);

        /* 将反馈转速写入 PID，计算控制量 */
        motor->position_pid.current = motor->position;
        double output = PID_Loop_Compute(&motor->position_pid, 0, M3508_ENCODER_RESOLUTION);

        /* 限幅到电流范围 [-16384, 16384] */
        if (output > M3508_CURRENT_MAX)  output = M3508_CURRENT_MAX;
        if (output < M3508_CURRENT_MIN)  output = M3508_CURRENT_MIN;

        /* 分入 LOW 组 (0~3) 或 HIGH 组 (4~7) */
        if (i < 4) {
            cur_low[i] = (int16_t)output;
        } else {
            cur_high[i - 4] = (int16_t)output;
        }
    }

    /* CAN 发送：一个 ID 控制一组 4 个电机 */
    M3508_SetCurrent(m3508_can, M3508_GROUP_LOW,  cur_low);
    M3508_SetCurrent(m3508_can, M3508_GROUP_HIGH, cur_high);
}
