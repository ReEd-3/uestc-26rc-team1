/**
 * @file    pid.c
 * @brief   GO8010 积分控制器实现（补 I）
 */
#include "pid.h"

void PID_Init(PID_t *pid, float ki, float dt, float integral_limit, float output_limit)
{
    pid->ki = ki;
    pid->dt = dt;
    pid->integral = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
}

float PID_Compute(PID_t *pid, float target, float current)
{
    float error = target - current;

    /* 积分 ∫e dt，限幅防饱和 */
    pid->integral += error * pid->dt;
    if (pid->integral_limit > 0)
    {
        if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
        if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    }

    /* 输出 = ki × ∫e dt，限幅 */
    float out = pid->ki * pid->integral;
    if (pid->output_limit > 0)
    {
        if (out >  pid->output_limit) out =  pid->output_limit;
        if (out < -pid->output_limit) out = -pid->output_limit;
    }

    return out;
}
