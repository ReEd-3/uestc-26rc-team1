#include "pid.h"


// PID控制器初始化函数，指定kp,ki,kd参数
void PID_Init(PID_t *pid, double Kp, double Ki, double Kd, double dt) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = dt;
    pid->integral = 0.0;
    pid->last_error = 0.0;
}

// PID计算函数，返回控制输出
double PID_Compute(PID_t *pid) {
    double error = pid->target - pid->current;  // 计算误差
    pid->integral += error * pid->dt;  // 积分误差
    double derivative = (error - pid->last_error) / pid->dt;  // 计算微分值
    pid->last_error = error;
    return pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
}

double PID_Loop_Compute(PID_t *pid, double high_lim, double low_lim) {
    if (high_lim < low_lim) {
        double temp = high_lim;
        high_lim = low_lim;
        low_lim = temp;
    }

    double dis_hl = high_lim - low_lim;  // 计算上下限差值
    double dis_hl_half = dis_hl / 2;  // 计算上下限差值的一半

    while (pid->target > high_lim) {
        pid->target -= dis_hl;
    }
    while (pid->target < low_lim) {
        pid->target += dis_hl;
    }
    while (pid->current > high_lim) {
        pid->current -= dis_hl;
    }
    while (pid->current < low_lim) {
        pid->current += dis_hl;
    }

    double error = pid->target - pid->current;  // 计算误差
    if (error > dis_hl_half) {
        error -= dis_hl;
    }
    else if (error < -dis_hl_half) {
        error += dis_hl;
    }

    pid->integral += error * pid->dt;  // 积分误差
    double derivative = (error - pid->last_error) / pid->dt;  // 计算微分值
    pid->last_error = error;
    return pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
}