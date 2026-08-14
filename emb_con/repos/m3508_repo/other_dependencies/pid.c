#include "pid.h"


// PID控制器初始化函数，指定kp,ki,kd参数
void PID_Init(PID_t *pid, double Kp, double Ki, double Kd, double dt) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = dt;
    pid->integral = 0.0;
    pid->integral_limit = -1.0;
    pid->last_error = 0.0;
    Int16_IIRFilter_Init(&pid->iir_filter, 1.0);  // 滤波默认直通，α 由上层设置
}

// 设置积分限幅
void PID_SetIntLim(PID_t *pid, double integral_limit) {
    pid->integral_limit = integral_limit;
}

// 设置滤波器参数
void PID_IIRFilter_SetAlpha(PID_t *pid, double alpha) {
    pid->iir_filter.filter_alpha = alpha;
}

// PID计算函数，返回控制输出
double PID_Compute(PID_t *pid) {
    double error = pid->target - pid->current;  // 计算误差
    pid->integral += error * pid->dt;  // 积分误差
    if (pid->integral_limit > 0) {  // 积分限幅：防止持续误差下积分无限累积
        if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
        if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    }
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
    if (pid->integral_limit > 0) {  // 积分限幅：防止持续误差下积分无限累积
        if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
        if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    }
    double derivative = (error - pid->last_error) / pid->dt;  // 计算微分值
    pid->last_error = error;
    return pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
}