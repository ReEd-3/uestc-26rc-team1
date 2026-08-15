#include "macnum.h"
#include <stdint.h>

#ifndef MACNUM_PI
#define MACNUM_PI 3.14159265358979323846
#endif

#define MACNUM_RPM_TO_RAD_S(rpm) ((rpm) * 2.0 * MACNUM_PI / 60.0)
#define MACNUM_RAD_S_TO_RPM(rad_s) ((rad_s) * 60.0 / (2.0 * MACNUM_PI))

// 麦轮初始化
void Macnum_Init (Macnum *mn,
                double wheel_radius,
                double half_wheelbase,
                double half_track) 
{   
    mn->wheel_radius = wheel_radius;
    mn->half_wheelbase = half_wheelbase;
    mn->half_track = half_track;

    mn->tar_vx = mn->tar_vy = mn->tar_omega = 0.0;
    mn->rea_vx = mn->rea_vy = mn->rea_omega = 0.0;
    for (int i = 0; i < 4; ++i) {
        mn->wheel_rpm[i] = 0.0;
        mn->rea_rpm[i] = 0.0;
    }
}

// 设置目标速度并解算出轮子的转速
void Macnum_SetTarget(Macnum *mn,
                double vx,
                double vy,
                double omega)
{
    mn->tar_vx = vx;
    mn->tar_vy = vy;
    mn->tar_omega = omega;
    double k = mn->half_track + mn->half_wheelbase;
    double r = mn->wheel_radius;
    double wheel_rad_s[4];
    // 解算轮子转速
    wheel_rad_s[0] = (vx - vy - k * omega) / r;
    wheel_rad_s[1] = (vx + vy + k * omega) / r;
    wheel_rad_s[2] = (vx + vy - k * omega) / r;
    wheel_rad_s[3] = (vx - vy + k * omega) / r;
    for (int i = 0; i < 4; ++i) {
        // 算出来是转动的角速度，转换成rpm转速
        mn->tar_rpm[i] = MACNUM_RAD_S_TO_RPM(wheel_rad_s[i]);
    }
}


void Macnum_StateUpdate(Macnum *mn, 
                double *rea_rpm) {
    double k = mn->half_track + mn->half_wheelbase;
    double r = mn->wheel_radius;
    double w[4];
    for (int i = 0; i < 4; ++i) {
        w[i] = MACNUM_RPM_TO_RAD_S(rea_rpm[i]);
    }
    // 计算真实的速度
    mn->rea_vx = r * (w[0] + w[1] + w[2] + w[3]) / 4.0;
    mn->rea_vy = r * (-w[0] + w[1] + w[2] - w[3]) / 4.0;
    mn->rea_omega = r * (-w[0] + w[1] - w[2] + w[3]) / (4.0 * k);
}