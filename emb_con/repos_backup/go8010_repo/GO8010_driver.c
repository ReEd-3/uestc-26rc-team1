/**
 * @file    GO8010_driver.c
 * @brief   GO-M8010-6 电机驱动实现
 */
#include "GO8010_driver.h"
#include <string.h>

/* 当前有在途命令的电机（半双工，同一时刻只可能有一个） */
static GO8010_Motor_t *current_motor = NULL;

/// @brief 手动按字节解析 16 字节电机反馈（不依赖位域布局，避免 armclang 与 gcc 打包差异）
/// @param buf  接收到的 16 字节原始数据
/// @param data 解析结果写入的电机反馈结构体
static void parse_feedback(const uint8_t *buf, MotorData_t *data)
{
    if (buf[0] != 0xFD || buf[1] != 0xEE)
    {
        data->correct = 0;
        return;
    }

    uint16_t crc = crc_ccitt(0, buf, 14);
    uint16_t crc_recv = (uint16_t)(buf[14] | (buf[15] << 8));
    if (crc != crc_recv)
    {
        data->correct = 0;
        data->bad_msg++;
        return;
    }

    data->motor_id = buf[2] & 0x0F;
    data->mode     = (buf[2] >> 4) & 0x07;
    data->Temp     = (int8_t)buf[11];

    uint16_t f = (uint16_t)(buf[12] | (buf[13] << 8));
    data->MError    = f & 0x07;
    data->footForce = (f >> 3) & 0x0FFF;

    int16_t torque = (int16_t)(buf[3] | (buf[4] << 8));
    int16_t speed  = (int16_t)(buf[5] | (buf[6] << 8));
    int32_t pos    = (int32_t)(buf[7] | (buf[8] << 8) | ((uint32_t)buf[9] << 16) | ((uint32_t)buf[10] << 24));

    data->T   = ((float)torque) / 256.0f;
    data->W   = ((float)speed / 256.0f) * 6.28318f;
    data->Pos = 6.28318f * ((float)pos) / 32768.0f;

    data->correct = 1;
}

void GO8010_Motor_Init(GO8010_Motor_t *motor, UART_HandleTypeDef *huart,
                       GPIO_TypeDef *de_port, uint16_t de_pin, uint8_t id)
{
    memset(motor, 0, sizeof(GO8010_Motor_t));
    motor->huart   = huart;
    motor->de_port = de_port;
    motor->de_pin  = de_pin;
    motor->id      = id;
    motor->last_rx_tick = HAL_GetTick();   /* 超时计时起点 */

    /* 方向脚默认接收态 */
    HAL_GPIO_WritePin(de_port, de_pin, GPIO_PIN_RESET);
}

void GO8010_Motor_SetCmd(GO8010_Motor_t *motor, uint8_t mode,
                         float T, float W, float Pos, float K_P, float K_W)
{
    motor->cmd.id   = motor->id;
    motor->cmd.mode = mode;
    motor->cmd.T    = T;
    motor->cmd.W    = W;
    motor->cmd.Pos  = Pos;
    motor->cmd.K_P  = K_P;
    motor->cmd.K_W  = K_W;
}

int GO8010_Motor_Send(GO8010_Motor_t *motor)
{
    modify_data(&motor->cmd);   /* 打包成 17 字节 */

    /* 停掉上一次可能未完成的接收 */
    HAL_UART_AbortReceive(motor->huart);

    /* 切到发送 */
    HAL_GPIO_WritePin(motor->de_port, motor->de_pin, GPIO_PIN_SET);

    if (HAL_UART_Transmit(motor->huart, (uint8_t *)&motor->cmd.motor_send_data,
                          sizeof(RIS_ControlData_t), 100) != HAL_OK)
    {
        HAL_GPIO_WritePin(motor->de_port, motor->de_pin, GPIO_PIN_RESET);
        return -1;
    }

    /* 等最后一个字节完全移出移位寄存器，避免过早切方向截断数据 */
    while (__HAL_UART_GET_FLAG(motor->huart, UART_FLAG_TC) == RESET)
    {
    }

    motor->tx_count++;

    /* 切回接收，武装接收中断（不等待回包，回包异步处理） */
    HAL_GPIO_WritePin(motor->de_port, motor->de_pin, GPIO_PIN_RESET);
    motor->rx_done = 0;
    current_motor  = motor;
    HAL_UART_Receive_IT(motor->huart, motor->rx_buf, sizeof(motor->rx_buf));

    return 0;
}

int GO8010_Motor_Poll(GO8010_Motor_t *motor)
{
    if (motor->rx_done)
    {
        motor->rx_done = 0;
        motor->last_rx_tick = HAL_GetTick();   /* 记录收到回包的时刻 */
        parse_feedback(motor->rx_buf, &motor->data);
        return 1;
    }
    return 0;
}

int GO8010_Motor_IsTimeout(GO8010_Motor_t *motor, uint32_t timeout_ms)
{
    return ((HAL_GetTick() - motor->last_rx_tick) > timeout_ms);
}

float GO8010_Motor_GetPos(const GO8010_Motor_t *motor)    { return motor->data.Pos; }
float GO8010_Motor_GetVel(const GO8010_Motor_t *motor)    { return motor->data.W; }
float GO8010_Motor_GetTorque(const GO8010_Motor_t *motor) { return motor->data.T; }
int   GO8010_Motor_GetTemp(const GO8010_Motor_t *motor)   { return motor->data.Temp; }
int   GO8010_Motor_GetError(const GO8010_Motor_t *motor)  { return motor->data.MError; }
int   GO8010_Motor_IsAlive(const GO8010_Motor_t *motor)   { return motor->data.correct; }

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (current_motor && huart == current_motor->huart)
    {
        current_motor->rx_done = 1;
    }
}
