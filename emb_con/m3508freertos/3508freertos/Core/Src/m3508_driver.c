#include "stm32h7xx_hal.h"
#include "fdcan_std.h"
#include "m3508_driver.h"
#include "pid.h"
#include "float.h"
#include "iir.h"

// 对总线上单个电机进行初始化
HAL_StatusTypeDef M3508_Init(M3508_HandleTypeDef *motor, FDCAN_HandleTypeDef *hfdcan, uint8_t can_id, M3508_PID_Mode mode, double max_speed) {
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
    motor->pid_mode = mode;
    motor->max_speed = max_speed;

    return HAL_OK; // 初始化成功
}

// 对控制电机的总线进行初始化（初始化后均为默认配置）
HAL_StatusTypeDef M3508_CAN_Init(M3508_CAN_All *m3508_can, uint8_t motor_ids, FDCAN_HandleTypeDef *hfdcan) {
    // 检测缓冲和滤波器数量
    if (hfdcan->Init.StdFiltersNbr < 8 ||
        hfdcan->Init.RxBuffersNbr  < 8) {
        return HAL_ERROR;
    }

    // 指定使用的can句柄
    m3508_can->hfdcan = hfdcan;

    // 指定滤波器配置
    FDCAN_FilterTypeDef sFilterConfig = {0};  // 这里必须置0，在arm compile环境下不置0编译会有问题
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

        // 初始化每个电机，主要是id, 默认速度环, 默认积分不限幅
        if (M3508_Init(&m3508_can->motors[i], hfdcan, i + 1, M3508_SPEEDPID_MODE, DBL_MAX) != HAL_OK) {
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

// 设置一整个CAN的电机电流
HAL_StatusTypeDef M3508_SetCurrent(M3508_CAN_All *m3508_can) {

    uint8_t data[8] = {0}; // 创建一个8字节的数据数组
    FDCAN_TxHeaderTypeDef CAN_TxHeader; // 创建发送报文头

    // 低ID电机电流设置
    for (int i = 0; i < 4; i++){
        if (m3508_can->motors[i].status == M3508_OFF) {
            continue;
        }
        if (m3508_can->cur_low[i] < M3508_CURRENT_MIN) {
            m3508_can->cur_low[i] = M3508_CURRENT_MIN; // 限制反向最大电流值
        } 
        else if (m3508_can->cur_low[i] > M3508_CURRENT_MAX) {
            m3508_can->cur_low[i] = M3508_CURRENT_MAX; // 限制最大电流值
        }
        data[2 * i] = (m3508_can->cur_low[i] >> 8) & 0xFF;  // 高字节
        data[2 * i + 1] = m3508_can->cur_low[i] & 0xFF;  // 低字节
    }

    HAL_FDCAN_StdDefault_TxHeaderInit(&CAN_TxHeader, M3508_CONTROL_ID_LOW, 8, m3508_can->hfdcan); // 初始化发送报文头
    if (HAL_FDCAN_Std_SendMessage(&CAN_TxHeader, m3508_can->hfdcan, data) != HAL_OK) {
        return HAL_ERROR;
    }; // 发送FDCAN报文

    // 高ID电机电流设置
    for (int i = 0; i < 4; i++){
        if (m3508_can->motors[i + 4].status == M3508_OFF) {
            continue;
        }
        if (m3508_can->cur_high[i] < M3508_CURRENT_MIN) {
            m3508_can->cur_high[i] = M3508_CURRENT_MIN; // 限制反向最大电流值
        } 
        else if (m3508_can->cur_high[i] > M3508_CURRENT_MAX) {
            m3508_can->cur_high[i] = M3508_CURRENT_MAX; // 限制最大电流值
        }
        data[2 * i] = (m3508_can->cur_high[i] >> 8) & 0xFF;  // 高字节
        data[2 * i + 1] = m3508_can->cur_high[i] & 0xFF;  // 低字节
    }

    HAL_FDCAN_StdDefault_TxHeaderInit(&CAN_TxHeader, M3508_CONTROL_ID_HIGH, 8, m3508_can->hfdcan); // 初始化发送报文头
    if (HAL_FDCAN_Std_SendMessage(&CAN_TxHeader, m3508_can->hfdcan, data) != HAL_OK) {
        return HAL_ERROR; 
    } // 发送FDCAN报文

    return HAL_OK;
}

// 读取总线上所有电机的状态
HAL_StatusTypeDef M3508_ReadStatus(M3508_CAN_All *m3508_can)
{
    HAL_StatusTypeDef result = HAL_ERROR;

    for (uint8_t i = 0; i < 8; i++)
    {
        // 跳过未使能的电机
        if (m3508_can->motors[i].status != M3508_ON) {
            continue;
        }

        uint8_t buf_idx = i;  // can_id=1 → RxBuffer[0], can_id=2 → RxBuffer[1], ...

        // 检查该电机专用缓冲区是否有新帧（硬件保证是该 ID 的帧）
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

// 给总线上所有电机设置速度
void M3508_SetSpeedTarget(M3508_CAN_All *m3508_can, double *target_rpm) {
    for(int i = 0; i < 8; i++) {
        if (m3508_can->motors[i].status == M3508_OFF) {
            continue;
        }
        m3508_can->motors[i].speed_pid.target = target_rpm[i];
    }
}

// 给单个电机初始化速度环参数
void M3508_SpeedPID_MotorInit(M3508_HandleTypeDef *motor, double Kp, double Ki, double Kd, double dt) {
    PID_Init(&motor->speed_pid, Kp, Ki, Kd, dt);
}

// 给总线所有电机初始化速度环参数
void M3508_SpeedPID_Init(M3508_CAN_All *m3508_can, double Kp, double Ki, double Kd, double dt) {
    for(int i = 0; i < 8; i++) {
        if (m3508_can->motors[i].status == M3508_OFF) {
            continue;
        }
        M3508_SpeedPID_MotorInit(&m3508_can->motors[i], Kp, Ki, Kd, dt);  // dt单位s
    }
}


// ============================================================
//  位置环 PID 控制
// ============================================================

// 控制同速度环PID

// 给总线上所有电机设置位置
void M3508_SetPositionTarget(M3508_CAN_All *m3508_can, double *target_rpm) {
    for(int i = 0; i < 8; i++) {
        if (m3508_can->motors[i].status == M3508_OFF) {
            continue;
        }
        m3508_can->motors[i].position_pid.target = target_rpm[i];
    }
}

// 给单个电机初始化位置环参数
void M3508_PositionPID_MotorInit(M3508_HandleTypeDef *motor, double Kp, double Ki, double Kd, double dt) {
    PID_Init(&motor->position_pid, Kp, Ki, Kd, dt);
}

// 给总线所有电机初始化位置环参数
void M3508_PositionPID_Init(M3508_CAN_All *m3508_can, double Kp, double Ki, double Kd, double dt) {
    for(int i = 0; i < 8; i++) {
        if (m3508_can->motors[i].status == M3508_OFF) {
            continue;
        }
        M3508_PositionPID_MotorInit(&m3508_can->motors[i], Kp, Ki, Kd, dt);  // dt单位s
    }
}


// ============================================================
//  通用 PID 更新
// ============================================================

// 对总线上所有电机进行PID更新
void M3508_PID_Update(M3508_CAN_All *m3508_can) {

    /* 读取所有电机反馈 */
    M3508_ReadStatus(m3508_can);

    /* 对每个电机按 PID 模式进行 PID 计算 */
    for (int i = 0; i < 8; i++) {

        if (m3508_can->motors[i].status != M3508_ON) {
            continue;  // 未启用的电机电流保持 0
        }

        M3508_HandleTypeDef *motor = &(m3508_can->motors[i]);

        double output = 0.0;

        /* 根据电机的PID模式分发计算 */
        switch (motor->pid_mode) {
            case M3508_SPEEDPID_MODE:  // 速度环模式
                motor->speed_pid.current = Int16_IIRFilter_Update(&motor->speed_pid.iir_filter, motor->speed);  // 滤波后的速度进环
                output = PID_Compute(&motor->speed_pid);
                break;

            case M3508_POSITIONPID_MODE:  // 位置环模式
                motor->position_pid.current = Int16_IIRFilter_Update(&motor->position_pid.iir_filter, motor->position);  // 滤波后的位置进环
                output = PID_Loop_Compute(&motor->position_pid, 0, M3508_ENCODER_RESOLUTION);
                break;

            case M3508_CASCADE_MODE:  // 串级模式
                motor->position_pid.current = Int16_IIRFilter_Update(&motor->position_pid.iir_filter, motor->position);
                output = PID_Loop_Compute(&motor->position_pid, 0, M3508_ENCODER_RESOLUTION);
                if (output >  motor->max_speed) output =  motor->max_speed;  // 内环目标限幅
                if (output < -motor->max_speed) output = -motor->max_speed;
                motor->speed_pid.target  = output;
                motor->speed_pid.current = Int16_IIRFilter_Update(&motor->speed_pid.iir_filter, motor->speed);  // 内环用滤波后速度
                output = PID_Compute(&motor->speed_pid);
                break;
        }

        /* 限幅到电流范围 [-16384, 16384] */
        if (output > M3508_CURRENT_MAX)  output = M3508_CURRENT_MAX;
        if (output < M3508_CURRENT_MIN)  output = M3508_CURRENT_MIN;

        /* 分入 LOW 组 (0~3) 或 HIGH 组 (4~7) */
        if (i < 4) {
            m3508_can->cur_low[i] = (int16_t)output;
        } else {
            m3508_can->cur_high[i - 4] = (int16_t)output;
        }
    }

    /* CAN 发送：一个 ID 控制一组 4 个电机 */
    // M3508_SetCurrent(m3508_can, M3508_GROUP_LOW,  cur_low);
    // M3508_SetCurrent(m3508_can, M3508_GROUP_HIGH, cur_high);

}

// 发送当前总线所有缓存的电流值
void M3508_CAN_CurrentUpdate(M3508_CAN_All *m3508_can) {
    M3508_SetCurrent(m3508_can);
}

// ============================================================
//  PID 模式切换,以及参数更新
// ============================================================

// 有三种模式，速度环模式，位置环模式，串级PID（位置环在外，速度环在内）
void M3508_PIDMode_Switch(M3508_HandleTypeDef *motor, M3508_PID_Mode mode) {

    if (motor->pid_mode == mode || motor->status == M3508_OFF) {
        return;
    }
    motor->pid_mode = mode;

    // 消除积分项和上次误差
    motor->speed_pid.integral = 0;
    motor->speed_pid.last_error = 0;
    motor->position_pid.integral = 0;
    motor->position_pid.last_error = 0;
    // 切换后旧环的滤波状态作废，重置（保留各自 alpha）
    Int16_IIRFilter_Init(&motor->speed_pid.iir_filter, motor->speed_pid.iir_filter.filter_alpha);
    Int16_IIRFilter_Init(&motor->position_pid.iir_filter, motor->position_pid.iir_filter.filter_alpha);
}

// 设置PID环的积分限幅
void M3508_PID_SetIntLim(M3508_HandleTypeDef *motor, M3508_PID_Mode mode, double integral_limit) {

    if (mode == M3508_SPEEDPID_MODE) {
        PID_SetIntLim(&motor->speed_pid, integral_limit);
    }
    else if (mode == M3508_POSITIONPID_MODE) {
        PID_SetIntLim(&motor->position_pid, integral_limit);
    }
}

// 设置指定 PID 环的低通滤波系数（0~1，1=直通不滤波）
void M3508_IIRFilter_SetAlpha(M3508_HandleTypeDef *motor, M3508_PID_Mode mode, double alpha) {

    if (motor == NULL) {
        return;
    }
    if (mode == M3508_SPEEDPID_MODE) {
        PID_IIRFilter_SetAlpha(&motor->speed_pid, alpha);
    }
    else if (mode == M3508_POSITIONPID_MODE) {
        PID_IIRFilter_SetAlpha(&motor->position_pid, alpha);
    }
}








