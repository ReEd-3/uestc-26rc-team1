#ifndef APP_H
#define APP_H

#include "chassis.h"
#include "m3508_driver.h"
#include "encoder_odo.h"
#include "chassis_task_1.h"
#include "uart_interact.h"
#include "stm32h7xx_hal.h"

typedef struct {
    Chassis        chassis;
    M3508_CAN_All  m3508;
    EncoderOdo     encoder;
    Chassis_Task1  task1;
    UartInteract   interact;

    /* 应用层状态 */
    volatile uint8_t task_paused;
    uint32_t         last_velocity_ms;
    volatile uint8_t arm_flag;
    volatile uint8_t rx_byte;
    uint32_t velocity_timeout_ms;
} App_Context;

typedef struct App_Config{
    /* 底盘配置 */
    double chassis_wheel_radius;   // 轮子半径
    double half_wheelbase;         // 轴距的一半
    double half_track;             // 轮距的一半
    double gear_ratio;             // 电机减速比

    double chassis_dt;             // 控制周期，单位 s，例如 0.001

    double max_vx;                 // 最大前进速度
    double max_vy;                 // 最大横移速度
    double max_omega;              // 最大自转角速度

    double pos_kp, pos_ki, pos_kd; // x/y 位置环 PID
    double yaw_kp, yaw_ki, yaw_kd; // yaw 角度环 PID
    double m3508_kp, m3508_ki, m3508_kd; // 电机 PID
    double m3508_iir_alpha;        // 电机低通滤波系数

    double tol_xy;                 // 到位判定：位置误差，单位 m
    double tol_yaw;                // 到位判定：角度误差，单位 rad

    /* 外设 */
    TIM_HandleTypeDef    *update1_tim;      // 任务更新定时器
    UART_HandleTypeDef   *interact_uart;    // 和上位机交互的串口
    FDCAN_HandleTypeDef  *chassis_m3508_hfdcan; // 控制底盘 M3508 的总线
    FDCAN_HandleTypeDef  *odo_hfdcan;       // 里程计总线

    /* 任务1参数 */
    double task1_move1_x;
    double task1_move1_y;
    double task1_move2_x;
    double task1_move2_y;
    double task1_move3_x;
    double task1_move3_y;

    /* 通信超时 */
    uint32_t velocity_timeout_ms;
} App_Config;

void Global_Init(App_Context *glb_app, const App_Config *cfg);

uint8_t App_IsPaused(void);
void App_CheckVelocityTimeout(uint32_t now_ms);
void App_PollEvents(void);

#endif
