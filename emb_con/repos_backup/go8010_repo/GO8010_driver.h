/**
 * @file    GO8010_driver.h
 * @brief   GO-M8010-6 电机驱动库（RS-485 半双工，非阻塞收发）
 */
#ifndef __GO8010_DRIVER_H
#define __GO8010_DRIVER_H

#include "gom_protocol.h"
#include "usart.h"

/* 减速比（物理常数，输出轴 = 转子 / 减速比） */
#define MOTOR_GEAR_RATIO  6.33f

/**
 * @brief 电机对象：一个电机一个实例，可多个电机共用一条 RS-485 总线
 */
typedef struct
{
    UART_HandleTypeDef *huart;   /* UART 句柄（需 4Mbps, 8N1） */
    GPIO_TypeDef       *de_port; /* RS-485 方向脚端口 */
    uint16_t            de_pin;  /* RS-485 方向脚引脚（DE，高=发送） */
    uint8_t             id;      /* 电机ID（0~14） */

    MotorCmd_t  cmd;             /* 命令（打包后 17 字节） */
    MotorData_t data;            /* 反馈（解析后） */

    uint8_t          rx_buf[16];   /* 接收缓冲区 */
    volatile uint8_t rx_done;      /* 接收完成标志（ISR 置位） */
    uint32_t         tx_count;     /* 成功发送计数 */
    uint32_t         last_rx_tick; /* 上次收到回包的时刻（超时检测用） */
} GO8010_Motor_t;

/**
 * @brief 初始化电机对象
 * @param motor   电机对象指针
 * @param huart   使用的 UART 句柄（4Mbps, 8N1）
 * @param de_port 方向脚端口（如 GPIOF）
 * @param de_pin  方向脚引脚（如 GPIO_PIN_14）
 * @param id      电机ID（0~14）
 */
void GO8010_Motor_Init(GO8010_Motor_t *motor, UART_HandleTypeDef *huart,
                       GPIO_TypeDef *de_port, uint16_t de_pin, uint8_t id);

/**
 * @brief 设置控制命令（参数均为减速前转子的量）
 * @param motor 电机对象
 * @param mode  工作模式（0=锁定，1=FOC闭环，2=编码器校准）
 * @param T     前馈力矩 N·m（-127.99~127.99）
 * @param W     期望角速度 rad/s（转子，-804~804）
 * @param Pos   期望位置 rad（转子）
 * @param K_P   位置刚度（0~25.599）
 * @param K_W   速度阻尼（0~25.599）
 */
void GO8010_Motor_SetCmd(GO8010_Motor_t *motor, uint8_t mode,
                         float T, float W, float Pos, float K_P, float K_W);

/**
 * @brief 非阻塞发送命令：打包 + 发送 + 切回接收并武装接收中断，立即返回
 * @param motor 电机对象
 * @return 0=发送成功，-1=发送失败
 * @note  发送阶段占用约 42.5µs；不等待电机回包，回包由中断异步接收
 */
int GO8010_Motor_Send(GO8010_Motor_t *motor);

/**
 * @brief 轮询是否收到新反馈（非阻塞）
 * @param motor 电机对象
 * @return 1=有新反馈（已解析进 motor->data），0=还没收到
 * @note  需在主循环/定时中断里周期调用；发送间隔要大于回包时间（约 200µs）
 */
int GO8010_Motor_Poll(GO8010_Motor_t *motor);

/**
 * @brief 检查通讯是否超时（超过 timeout_ms 没收到回包）
 * @param motor 电机对象
 * @param timeout_ms 超时时间（毫秒）
 * @return 1=超时（失联），0=正常
 */
int GO8010_Motor_IsTimeout(GO8010_Motor_t *motor, uint32_t timeout_ms);

/* ---- 反馈读取 ---- */
float GO8010_Motor_GetPos(const GO8010_Motor_t *motor);
float GO8010_Motor_GetVel(const GO8010_Motor_t *motor);
float GO8010_Motor_GetTorque(const GO8010_Motor_t *motor);
int   GO8010_Motor_GetTemp(const GO8010_Motor_t *motor);
int   GO8010_Motor_GetError(const GO8010_Motor_t *motor);
int   GO8010_Motor_IsAlive(const GO8010_Motor_t *motor);   /* 最近一次反馈 CRC 正确 */

#endif
