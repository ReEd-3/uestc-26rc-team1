#include "macnum.h"
#include <math.h>
#include <stdint.h>

#ifndef MACNUM_PI
#define MACNUM_PI 3.14159265358979323846
#endif

#define MACNUM_RPM_TO_RAD_S(rpm) ((rpm) * 2.0 * MACNUM_PI / 60.0)
#define MACNUM_RAD_S_TO_RPM(rad_s) ((rad_s) * 60.0 / (2.0 * MACNUM_PI))
#define MACNUM_WHEEL_COUNT 4
#define MACNUM_COUNTS_PER_REV 8192


// 麦轮初始化
void Macnum_Init (Macnum *mn,
                double wheel_radius,
                double half_wheelbase,
                double half_track,
                double gear_ratio
            ) 
{   
    mn->wheel_radius = wheel_radius;
    mn->half_wheelbase = half_wheelbase;
    mn->half_track = half_track;
    mn->gear_ratio = gear_ratio;
    mn->encoder_counts = MACNUM_COUNTS_PER_REV;

    mn->tar_vx = mn->tar_vy = mn->tar_omega = 0.0;
    // mn->rea_vx = mn->rea_vy = mn->rea_omega = 0.0;
    // mn->rea_x = mn->rea_y = mn->yaw = 0.0;

    for (int i = 0; i < 4; ++i) {
        mn->tar_rpm[i] = 0.0;
        mn->rotation[i] = 1;  // 默认正装
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
        // 输出的是电机轴 RPM：轮子 RPM 必须乘减速比，M3508 速度反馈是电机轴转速
        mn->tar_rpm[i] = MACNUM_RAD_S_TO_RPM(wheel_rad_s[i]) * mn->gear_ratio;
    }
}

