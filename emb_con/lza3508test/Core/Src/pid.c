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