#ifndef PID_H
#define PID_H

#include "stm32h7xx_hal.h"

typedef struct {
    double Kp;  // 比例增益
    double Ki;  // 积分增益
    double Kd;  // 微分增益
    double dt;  // 时间间隔
    double integral;       // 积分值
    double last_error;     // 上一次误差值
    double target;
    double current;
} PID_t;

void PID_Init(PID_t *pid, double Kp, double Ki, double Kd, double dt);
double PID_Compute(PID_t *pid);

#endif