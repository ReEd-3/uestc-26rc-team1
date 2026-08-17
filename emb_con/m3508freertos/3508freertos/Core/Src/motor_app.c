#include "motor_app.h"
#include "main.h"
#include <stdio.h>

static double target_rpm[8] = {0};// 8 个电机的目标位置（计数值，0~8191）
static M3508_CAN_All m3508_can_1;// 8 个 M3508 电机的句柄数组，使用 CAN1
static char vofa_buf[64];// VOFA+ 遥测缓冲区
volatile uint32_t motor_pid_count = 0;// 调试：PID 任务唤醒计数（应约 500/s），全局便于 Watch 查看
volatile uint32_t motor_cur_count = 0;// 调试：发电流任务唤醒计数（应约 1000/s），全局便于 Watch 查看

/*  电机初始化（串级模式：位置环在外，速度环在内） */
void Motor_AppInit(void) {
    M3508_CAN_Init(&m3508_can_1, (1 << 4), &hfdcan1);// 只初始化 ID=5 的电机

    M3508_SpeedPID_Init(&m3508_can_1, 4, 2, 0.02, 0.002);// 速度环 PID 参数设置
    M3508_PositionPID_Init(&m3508_can_1, 3.5, 0.05, 0.02, 0.002);// 位置环 PID 参数设置

    M3508_PID_SetIntLim(&m3508_can_1.motors[4], M3508_SPEEDPID_MODE, 400);// 积分限幅设置
    M3508_PID_SetIntLim(&m3508_can_1.motors[4], M3508_POSITIONPID_MODE, 500);// 积分限幅设置

    M3508_IIRFilter_SetAlpha(&m3508_can_1.motors[4], M3508_SPEEDPID_MODE, 0.8);// 指定环的低通滤波系数设置（0~1，1=直通）
    M3508_IIRFilter_SetAlpha(&m3508_can_1.motors[4], M3508_POSITIONPID_MODE, 1);// 指定环的低通滤波系数设置（0~1，1=直通）

    target_rpm[4] = 0; // 位置目标初始 0，首拍会吸附到当前位置
    M3508_SetPositionTarget(&m3508_can_1, target_rpm);// 写入位置环目标
    M3508_PIDMode_Switch(&m3508_can_1.motors[4], M3508_CASCADE_MODE);// 串级模式
    m3508_can_1.motors[4].max_speed = 350;// 串级内环速度限幅（RPM）
}

/* PID 任务调用 */
void Motor_PIDUpdate(void) {
    motor_pid_count++;  // 调试：PID 任务被唤醒一次

    static uint8_t pos_init = 0;
    if (!pos_init) {  // 首拍：位置目标吸附到当前位置，避免初始误差跳变
        pos_init = 1;
        target_rpm[4] = m3508_can_1.motors[4].position;
        M3508_SetPositionTarget(&m3508_can_1, target_rpm);// 同步进 position_pid.target
    }
    M3508_PID_Update(&m3508_can_1); // 串级：读反馈 + 双环计算
}

/* 发送缓存电流任务调用*/
void Motor_CurUpdate(void) {
    motor_cur_count++;  // 调试：发电流任务被唤醒一次

    target_rpm[4] += 20;  // 位置目标每拍递增 20 计数
    if (target_rpm[4] > M3508_ENCODER_RESOLUTION) {
        target_rpm[4] -= M3508_ENCODER_RESOLUTION;
    }
    M3508_SetPositionTarget(&m3508_can_1, target_rpm);// 位置目标更新
    M3508_CAN_CurrentUpdate(&m3508_can_1);  // 发送电流
}

/*  VOFA+ 遥测  */
void VOFA_Send(void) {
    int len = sprintf(vofa_buf, "%d,%d,%d,%d\n",
        (int)m3508_can_1.motors[4].position_pid.target,  // 目标位置
        (int)m3508_can_1.motors[4].position,             // 当前位置
        (int)m3508_can_1.motors[4].speed_pid.target,     // 目标速度（RPM）
        (int)m3508_can_1.motors[4].current               // 电流
      );
    HAL_UART_Transmit_IT(&huart3, (uint8_t *)vofa_buf, len);
}
