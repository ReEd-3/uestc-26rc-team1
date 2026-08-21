#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>
#include "macnum.h"
#include "m3508_driver.h"
#include "pid.h"
#include "encoder_odo.h"

typedef enum {
    CHASSIS_MODE_STOP,
    CHASSIS_MODE_VELOCITY,       // 直接速度控制
    CHASSIS_MODE_RELATIVE_MOVE,  // 相对位置
    CHASSIS_MODE_ABSOLUTE_MOVE,  // 绝对位置
} Chassis_Mode;

typedef struct {
    double wheel_radius;     // 轮子半径
    double half_wheelbase;   // 轴距的一半
    double half_track;       // 轮距的一半
    double gear_ratio;       // 电机减速比

    double dt;               // 控制周期，单位 s，例如 0.001

    double max_vx;           // 最大前进速度
    double max_vy;           // 最大横移速度
    double max_omega;        // 最大自转角速度

    double pos_kp, pos_ki, pos_kd;   // x/y 位置环 PID
    double yaw_kp, yaw_ki, yaw_kd;   // yaw 角度环 PID

    double tol_xy;           // 到位判定：位置误差，单位 m
    double tol_yaw;          // 到位判定：角度误差，单位 rad
} Chassis_Config;

typedef struct {
    M3508_CAN_All *m3508;    // 电机/CAN 句柄
    Macnum mn;               // 麦轮解算
    EncoderOdo *eo;           // 码盘

    Chassis_Mode mode;

    // 速度模式目标
    double tar_vx, tar_vy, tar_omega;

    // 位置模式目标（统一保存为绝对目标）
    double target_x, target_y, target_yaw;

    double dt;               // 底盘PID控制周期，单位 s
    double max_vx, max_vy, max_omega;
    double tol_xy, tol_yaw;

    PID_t pid_x, pid_y, pid_yaw;
} Chassis;

void Chassis_Init(Chassis *ch, M3508_CAN_All *m3508, EncoderOdo *eo, const Chassis_Config *cfg);
void Chassis_ResetEncoderPose(Chassis *ch);
void Chassis_SetPose(Chassis *ch, double x, double y, double yaw);
void Chassis_ResetPose(Chassis *ch, const uint16_t encoder_now[4]);
void Chassis_SetVelocity(Chassis *ch, double vx, double vy, double omega);
void Chassis_MoveRelative(Chassis *ch, double dx, double dy, double dyaw);
void Chassis_MoveAbsolute(Chassis *ch, double x, double y, double yaw);
void Chassis_Stop(Chassis *ch);
void Chassis_Update(Chassis *ch);
uint8_t Chassis_Arrived(Chassis *ch);

#endif