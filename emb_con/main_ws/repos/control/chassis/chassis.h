#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>
#include "macnum.h"
#include "m3508_driver.h"
#include "pid.h"
#include "encoder_odo.h"
#include "yis506.h"

typedef struct App_Config App_Config;

typedef enum {
    CHASSIS_MODE_STOP,
    CHASSIS_MODE_VELOCITY,       // 直接速度控制
    CHASSIS_MODE_RELATIVE_MOVE,  // 相对位置
    CHASSIS_MODE_ABSOLUTE_MOVE,  // 绝对位置
} Chassis_Mode;

typedef struct {
    M3508_CAN_All *m3508;    // 电机/CAN 句柄
    Macnum *mn;               // 麦轮解算
    EncoderOdo *eo;           // 码盘
    Yis506 *yis;              // IMU  

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

void Chassis_Init(Chassis *ch, M3508_CAN_All *m3508, EncoderOdo *eo, Macnum *mn, Yis506 *yis, const App_Config *cfg);
void Chassis_ResetEncoderPose(Chassis *ch);
void Chassis_SetPose(Chassis *ch, double x, double y, double yaw);
void Chassis_ResetPose(Chassis *ch, const uint16_t encoder_now[4]);
void Chassis_SetVelocity(Chassis *ch, double vx, double vy, double omega);
void Chassis_MoveRelative(Chassis *ch, double dx, double dy, double dyaw);
void Chassis_MoveAbsolute(Chassis *ch, double x, double y, double yaw);
void Chassis_FollowTarget(Chassis *ch, double x, double y, double yaw);  // 不重置PID，用于轨迹跟踪
void Chassis_Stop(Chassis *ch);
void Chassis_Update(Chassis *ch);
uint8_t Chassis_Arrived(Chassis *ch);

#endif