#ifndef CHASSIS_H
#define CHASSIS_H

#include "stdint.h"
#include "stdbool.h"
#include "m3508_driver.h"
#include "pid.h"
#include "macnum.h"

#define WHEEL_RADIUS        0.04f
#define WHEEL_BASE_X        0.20f
#define WHEEL_BASE_Y        0.15f

typedef struct {
    float vx, vy, omega;
    float real_vx, real_vy, real_omega;
    Macnum macnum;
    PID_t pid_x, pid_y, pid_rot;
    bool point_reached;
    float arrive_pos_err;
    float arrive_angle_err;
    uint32_t point_tick;
    uint32_t point_tick_threshold;

    // 循线 PID 参数
    float line_kp;
    float line_ki;          // 新增：积分系数
    float line_kd;
    float line_integral;    // 新增：积分累加
    float line_integral_max;// 新增：积分限幅
    float last_line_error;
    float line_forward_speed;
} Chassis_t;

extern Chassis_t chassis;

void Chassis_Init(void);
void Chassis_SetBodyVelocity(float vx, float vy, float omega);
bool Chassis_MoveToPoint(float target_x, float target_y, float target_yaw,
                         float current_x, float current_y, float current_yaw);
void Chassis_LineFollow(float line_error);
void Chassis_Stop(void);
void Chassis_UpdateActualVelocity(double *actual_rpm);

#endif
