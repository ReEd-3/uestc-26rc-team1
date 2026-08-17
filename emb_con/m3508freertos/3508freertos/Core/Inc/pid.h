#ifndef PID_H
#define PID_H

#include "stm32h7xx_hal.h"
#include "iir.h"

typedef struct {
    double Kp;  // 比例增益
    double Ki;  // 积分增益
    double Kd;  // 微分增益
    double dt;  // 时间间隔
    double integral;       // 积分值
    double integral_limit; // 积分限幅（>0 生效；<=0 表示不限幅）
    double last_error;     // 上一次误差值
    double target;
    double current;
    Int16_IIR iir_filter;  // 低通滤波器
} PID_t;

void PID_Init(PID_t *pid, double Kp, double Ki, double Kd, double dt);
void PID_SetIntLim(PID_t *pid, double integral_limit);
void PID_IIRFilter_SetAlpha(PID_t *pid, double alpha);
double PID_Compute(PID_t *pid);
double PID_Loop_Compute(PID_t *pid, double high_lim, double low_lim);

#endif







