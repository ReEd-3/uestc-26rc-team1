#include "macnum.h"
#include "stm32h7xx_hal.h"
#include <math.h>
#include <stdint.h>

#ifndef MACNUM_PI
#define MACNUM_PI 3.14159265358979323846
#endif

#define MACNUM_RPM_TO_RAD_S(rpm) ((rpm) * 2.0 * MACNUM_PI / 60.0)
#define MACNUM_RAD_S_TO_RPM(rad_s) ((rad_s) * 60.0 / (2.0 * MACNUM_PI))
#define MACNUM_WHEEL_COUNT 4
#define MACNUM_COUNTS_PER_REV 8192

// 计算前后编码器之差
static int32_t Macnum_Odo_WrapDelta(uint16_t current, int32_t last)
{
    int32_t delta = (int32_t)current - last;

    if (delta > MACNUM_COUNTS_PER_REV / 2) {
        delta -= MACNUM_COUNTS_PER_REV;
    } else if (delta < -MACNUM_COUNTS_PER_REV / 2) {
        delta += MACNUM_COUNTS_PER_REV;
    }

    return delta;
}

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
    mn->rea_vx = mn->rea_vy = mn->rea_omega = 0.0;
    mn->rea_x = mn->rea_y = mn->yaw = 0.0;

    for (int i = 0; i < 4; ++i) {
        mn->tar_rpm[i] = 0.0;
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

// 更新速度状态
void Macnum_RPMStateUpdate(Macnum *mn, 
                double *rea_rpm) 
{
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

// 初始调用确定电机初始状态
void Macnum_PositionReset (Macnum *mn, uint16_t *encoder_now)
{
    mn->tar_vx = mn->tar_vy = mn->tar_omega = 0.0;
    mn->rea_vx = mn->rea_vy = mn->rea_omega = 0.0;
    mn->rea_x = mn->rea_y = mn->yaw = 0.0;
    for (uint8_t i = 0; i < MACNUM_WHEEL_COUNT; i++) {
        mn->raw_encoder[i] = encoder_now[i];
    }
}

// 更新里程计状态
void Macnum_PositionStateUpdate(Macnum *mn, uint16_t *encoder_raw)
{
    double wheel_displacement[MACNUM_WHEEL_COUNT];
    double circumference = 2.0 * MACNUM_PI * mn->wheel_radius;

    /* 编码器增量 -> 轮子位移，除以减速比得到输出轴圈数对应的位移 */
    for (int i = 0; i < MACNUM_WHEEL_COUNT; ++i) {
        int32_t delta = Macnum_Odo_WrapDelta((uint16_t)encoder_raw[i],
                                             (int32_t)mn->raw_encoder[i]);
        wheel_displacement[i] = (double)delta / mn->encoder_counts *
                                circumference / mn->gear_ratio;
        mn->raw_encoder[i] = encoder_raw[i];
    }

    /*
     * 麦轮正运动学，轮子顺序按现有 macnum 约定：
     * 0=FL, 1=FR, 2=RL, 3=RR
     */
    double d_local_x = (wheel_displacement[0] + wheel_displacement[1] +
                        wheel_displacement[2] + wheel_displacement[3]) / 4.0;

    double d_local_y = (-wheel_displacement[0] + wheel_displacement[1] +
                        wheel_displacement[2] - wheel_displacement[3]) / 4.0;

    double d_theta = (-wheel_displacement[0] + wheel_displacement[1] -
                      wheel_displacement[2] + wheel_displacement[3]) /
                     (4.0 * (mn->half_wheelbase + mn->half_track));

    /* 局部位移转换到全局坐标 */
    double cos_yaw = cos(mn->yaw);
    double sin_yaw = sin(mn->yaw);

    mn->rea_x += d_local_x * cos_yaw - d_local_y * sin_yaw;
    mn->rea_y += d_local_x * sin_yaw + d_local_y * cos_yaw;
    mn->yaw += d_theta;

    /* 归一化 yaw 到 [-pi, pi] */
    if (mn->yaw > MACNUM_PI) {
        mn->yaw -= 2.0 * MACNUM_PI;
    } else if (mn->yaw < -MACNUM_PI) {
        mn->yaw += 2.0 * MACNUM_PI;
    }
}

