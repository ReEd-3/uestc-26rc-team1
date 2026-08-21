/**
 * @file    GO8010_driver.h
 * @brief   GO-M8010-6 关节电机驱动库（RS-485 半双工，阻塞收发，7 种控制模式）
 *
 * 功能：宇树 GO-M8010-6 电机的通讯驱动，提供 7 种控制模式、阻塞收发、反馈读取。
 *       硬件通过 Init 传入，换板子/换引脚/换串口只需改 Init 那一行，方便移植。
 *
 * 硬件要求：
 *   - 1 个 UART，波特率 4Mbps，8N1
 *   - 1 个 GPIO 作为 RS-485 方向脚（DE：高=发送，低=接收）
 *   - RS-485 收发器需支持 4Mbps（如 SP3485/MAX3485；MAX485 只有 2.5Mbps 不行）
 *
 * 快速上手：
 *   GO8010_Motor_t motor;
 *   GO8010_Motor_Init(&motor, &huart9, GPIOF, GPIO_PIN_14, 1);   // UART/DE脚/ID
 *   GO8010_Motor_SetVelocity(&motor, 6.28f*6.33f, 0.02f);        // 选一种模式
 *   while (1) {
 *       GO8010_Motor_SendRecv(&motor, 10);                       // 阻塞收发（10ms超时）
 *       float pos = GO8010_Motor_GetPos(&motor);                 // 读反馈
 *       HAL_Delay(1);                                            // 1kHz
 *   }
 *
 * 注意事项：
 *   - W/Pos 是「减速前转子」的量，输出轴 = 转子 / 减速比(6.33)
 *   - 需持续发命令（电机有通讯看门狗，1kHz 合适，别只发一次）
 *   - 半双工：同一时刻只能一个电机在收发
 */
#ifndef __GO8010_DRIVER_H
#define __GO8010_DRIVER_H

#include "gom_protocol.h"
#include "usart.h"

/* 减速比（物理常数，输出轴 = 转子 / 减速比） */
#define MOTOR_GEAR_RATIO  6.33f

/**
 * @brief 控制模式（对应手册 7.1~7.7 + 停止）
 */
typedef enum {
    GO8010_MODE_STOP = 0,       /* 停止/锁定（mode=0） */
    GO8010_MODE_VELOCITY,       /* 速度模式：W + K_W */
    GO8010_MODE_POSITION,       /* 位置模式：Pos + K_P + K_W */
    GO8010_MODE_DAMPING,        /* 阻尼模式：仅 K_W（W=0） */
    GO8010_MODE_TORQUE,         /* 力矩模式：仅 T */
    GO8010_MODE_ZERO_TORQUE,    /* 零力矩模式：全 0 */
    GO8010_MODE_HYBRID,         /* 力位混合模式：T+W+Pos+K_P+K_W */
} GO8010_Mode_t;

/**
 * @brief 电机对象：一个电机一个实例，可多个电机共用一条 RS-485 总线
 */
typedef struct
{
    UART_HandleTypeDef *huart;   /* UART 句柄（需 4Mbps, 8N1） */
    GPIO_TypeDef       *de_port; /* RS-485 方向脚端口（如 GPIOF） */
    uint16_t            de_pin;  /* RS-485 方向脚引脚（如 GPIO_PIN_14，高=发送低=接收） */
    uint8_t             id;      /* 电机ID（0~14，需与电机实际配置一致） */

    MotorCmd_t  cmd;             /* 命令（内部打包用） */
    MotorData_t data;            /* 反馈（SendRecv 后解析进这里） */

    uint8_t  rx_buf[16];         /* 接收缓冲区（内部用） */
    uint32_t tx_count;           /* 成功发送计数（调试观察用） */
} GO8010_Motor_t;

/**
 * @brief 初始化电机对象
 * @param motor   电机对象指针
 * @param huart   使用的 UART 句柄（需已配置成 4Mbps, 8N1）
 * @param de_port 方向脚端口（如 GPIOF）
 * @param de_pin  方向脚引脚（如 GPIO_PIN_14）
 * @param id      电机ID（0~14，需与电机实际 ID 一致）
 */
void GO8010_Motor_Init(GO8010_Motor_t *motor, UART_HandleTypeDef *huart,
                       GPIO_TypeDef *de_port, uint16_t de_pin, uint8_t id);

/* ================= 7 种控制模式 =================
 * 参数均为「减速前转子」的量：输出轴 = 转子 / 6.33
 * ================================================ */

/** 停止/锁定（mode=0，电机锁死） */
void GO8010_Motor_Stop(GO8010_Motor_t *motor);

/**
 * @brief 速度模式：电机以恒定速度旋转（手册 7.1/7.3）
 * @param w_rotor 目标速度（转子 rad/s；输出轴速度 ×6.33）
 * @param k_w     速度比例系数 K_W（0~25.599，典型 0.02，越大响应越快）
 */
void GO8010_Motor_SetVelocity(GO8010_Motor_t *motor, float w_rotor, float k_w);

/**
 * @brief 位置模式：输出轴稳定在目标位置（手册 7.2，电机内置 PD）
 * @param pos_rotor 目标位置（转子 rad；输出轴位置 ×6.33）
 * @param k_p       位置刚度 K_P（0~25.599，典型 0.2，越大越"硬"）
 * @param k_w       速度阻尼 K_W（0~25.599，0 无阻尼，震荡时加大）
 */
void GO8010_Motor_SetPosition(GO8010_Motor_t *motor, float pos_rotor, float k_p, float k_w);

/**
 * @brief 阻尼模式：保持速度 0，被外力转时产生阻抗（手册 7.4）
 * @param k_w 阻尼系数 K_W（0~25.599，典型 0.02）
 */
void GO8010_Motor_SetDamping(GO8010_Motor_t *motor, float k_w);

/**
 * @brief 力矩模式：持续输出恒定力矩（手册 7.5）
 * @param torque 目标力矩 T（N·m，-127.99~127.99；空载会持续加速）
 */
void GO8010_Motor_SetTorque(GO8010_Motor_t *motor, float torque);

/** 零力矩模式：力矩为 0，主动抵抗自身摩擦（手册 7.6） */
void GO8010_Motor_SetZeroTorque(GO8010_Motor_t *motor);

/**
 * @brief 力位混合模式：同时给前馈力矩+目标速度+目标位置（手册 7.7）
 * @param T   前馈力矩（N·m）
 * @param W   目标速度（转子 rad/s）
 * @param Pos 目标位置（转子 rad）
 * @param K_P 位置刚度
 * @param K_W 速度阻尼
 */
void GO8010_Motor_SetHybrid(GO8010_Motor_t *motor, float T, float W, float Pos,
                            float K_P, float K_W);

/* ================= 收发 ================= */

/**
 * @brief 阻塞收发：打包 + 发送 + 阻塞接收 + 解析，一次完成
 * @param motor 电机对象
 * @param timeout_ms 等待回包的超时时间（毫秒，典型 10）
 * @return 1=收到有效回包，0=超时/发送失败
 * @note  阻塞约 150µs（回包时间）；1kHz 下占用 15% 周期，单电机够用
 */
int GO8010_Motor_SendRecv(GO8010_Motor_t *motor, uint32_t timeout_ms);

/* ================= 反馈读取 ================= */

/** 实际位置（转子 rad；输出轴 = 返回值 / 6.33） */
float GO8010_Motor_GetPos(const GO8010_Motor_t *motor);
/** 实际速度（转子 rad/s；输出轴 = 返回值 / 6.33） */
float GO8010_Motor_GetVel(const GO8010_Motor_t *motor);
/** 实际力矩（N·m） */
float GO8010_Motor_GetTorque(const GO8010_Motor_t *motor);
/** 电机温度（℃） */
int   GO8010_Motor_GetTemp(const GO8010_Motor_t *motor);
/** 错误码：0=正常，1=过热，2=过流，3=过压，4=编码器故障，5=母线欠压，6=绕组过热 */
int   GO8010_Motor_GetError(const GO8010_Motor_t *motor);
/** 最近一次反馈 CRC 是否正确（1=通讯正常） */
int   GO8010_Motor_IsAlive(const GO8010_Motor_t *motor);

#endif
