/**
 * @file    GO8010_driver.c
 * @brief   GO-M8010-6 电机驱动实现
 */
#include "GO8010_driver.h"
#include <string.h>

/* 手动按字节解析 16 字节反馈（不依赖位域布局，避免 armclang/gcc 打包差异） */
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

/* 内部：设置原始命令（各模式函数都调它） */
static void GO8010_Motor_SetCmd(GO8010_Motor_t *motor, uint8_t mode,
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

void GO8010_Motor_Init(GO8010_Motor_t *motor, UART_HandleTypeDef *huart,
                       GPIO_TypeDef *de_port, uint16_t de_pin, uint8_t id)
{
    memset(motor, 0, sizeof(GO8010_Motor_t));
    motor->huart   = huart;
    motor->de_port = de_port;
    motor->de_pin  = de_pin;
    motor->id      = id;

    /* 方向脚默认接收态 */
    HAL_GPIO_WritePin(de_port, de_pin, GPIO_PIN_RESET);
}

/* ============ 7 种控制模式 ============ */

void GO8010_Motor_Stop(GO8010_Motor_t *motor)
{
    GO8010_Motor_SetCmd(motor, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void GO8010_Motor_SetVelocity(GO8010_Motor_t *motor, float w_rotor, float k_w)
{
    GO8010_Motor_SetCmd(motor, 1, 0.0f, w_rotor, 0.0f, 0.0f, k_w);
}

void GO8010_Motor_SetPosition(GO8010_Motor_t *motor, float pos_rotor, float k_p, float k_w)
{
    GO8010_Motor_SetCmd(motor, 1, 0.0f, 0.0f, pos_rotor, k_p, k_w);
}

void GO8010_Motor_SetDamping(GO8010_Motor_t *motor, float k_w)
{
    GO8010_Motor_SetCmd(motor, 1, 0.0f, 0.0f, 0.0f, 0.0f, k_w);
}

void GO8010_Motor_SetTorque(GO8010_Motor_t *motor, float torque)
{
    GO8010_Motor_SetCmd(motor, 1, torque, 0.0f, 0.0f, 0.0f, 0.0f);
}

void GO8010_Motor_SetZeroTorque(GO8010_Motor_t *motor)
{
    GO8010_Motor_SetCmd(motor, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void GO8010_Motor_SetHybrid(GO8010_Motor_t *motor, float T, float W, float Pos,
                            float K_P, float K_W)
{
    GO8010_Motor_SetCmd(motor, 1, T, W, Pos, K_P, K_W);
}

/* ============ 阻塞收发 ============ */

int GO8010_Motor_SendRecv(GO8010_Motor_t *motor, uint32_t timeout_ms)
{
    modify_data(&motor->cmd);   /* 打包成 17 字节 */

    HAL_UART_AbortReceive(motor->huart);   /* 复位接收状态 */

    /* 切到发送 */
    HAL_GPIO_WritePin(motor->de_port, motor->de_pin, GPIO_PIN_SET);

    if (HAL_UART_Transmit(motor->huart, (uint8_t *)&motor->cmd.motor_send_data,
                          sizeof(RIS_ControlData_t), 100) != HAL_OK)
    {
        HAL_GPIO_WritePin(motor->de_port, motor->de_pin, GPIO_PIN_RESET);
        return 0;
    }

    /* 等最后一个字节完全移出移位寄存器，避免过早切方向截断数据 */
    while (__HAL_UART_GET_FLAG(motor->huart, UART_FLAG_TC) == RESET)
    {
    }

    motor->tx_count++;

    /* 切回接收 */
    HAL_GPIO_WritePin(motor->de_port, motor->de_pin, GPIO_PIN_RESET);

    /* 清掉发送期间 RX 脚悬空可能产生的杂散字节/错误标志（防阻塞接收被干扰） */
    __HAL_UART_CLEAR_OREFLAG(motor->huart);   /* 清过载 ORE（内部读 RDR，顺带清 RXNE） */
    __HAL_UART_CLEAR_FEFLAG(motor->huart);    /* 清帧错误 FE */

    /* 阻塞接收 16 字节回包 */
    if (HAL_UART_Receive(motor->huart, motor->rx_buf, sizeof(motor->rx_buf), timeout_ms) != HAL_OK)
    {
        motor->data.timeout++;
        return 0;
    }

    parse_feedback(motor->rx_buf, &motor->data);   /* 解析 */
    return motor->data.correct;
}

/* ============ 反馈读取 ============ */

float GO8010_Motor_GetPos(const GO8010_Motor_t *motor)    { return motor->data.Pos; }
float GO8010_Motor_GetVel(const GO8010_Motor_t *motor)    { return motor->data.W; }
float GO8010_Motor_GetTorque(const GO8010_Motor_t *motor) { return motor->data.T; }
int   GO8010_Motor_GetTemp(const GO8010_Motor_t *motor)   { return motor->data.Temp; }
int   GO8010_Motor_GetError(const GO8010_Motor_t *motor)  { return motor->data.MError; }
int   GO8010_Motor_IsAlive(const GO8010_Motor_t *motor)   { return motor->data.correct; }
