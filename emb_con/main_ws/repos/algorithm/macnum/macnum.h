#ifndef MACNUM_H
#define MACNUM_H

#include <stdint.h>

typedef struct {
    double wheel_radius;  // 轮子半径
    double half_wheelbase;  // 轴距的一半
    double half_track;  // 轮距的一半
    double gear_ratio;  // 电机减速比
    double tar_vx, tar_vy, tar_omega;  // 目标速度,转角
    double tar_rpm[4];  // 分别是FL, FR, RL, RR的电机轴RPM（已乘减速比）
    // double rea_vx, rea_vy, rea_omega;  // 真实的运动状态
    // double rea_x, rea_y, yaw;  // 全局位置和姿态
    double raw_encoder[4];  // 编码器
    int8_t rotation[4];    // 轮子安装方向: 1=正装, -1=反装
    uint32_t encoder_counts;
} Macnum;  // 麦轮解算句柄，专用来解算

void Macnum_Init (Macnum *mn, double wheel_radius, double half_wheelbase, double half_track, double gear_ratio);
void Macnum_SetTarget (Macnum *mn, double vx, double vy, double omega);
// void Macnum_RPMStateUpdate (Macnum *mn, double *rea_rpm);
// void Macnum_PositionReset (Macnum *mn, uint16_t *encoder_now);
// void Macnum_PositionStateUpdate (Macnum *mn, uint16_t *encoder_raw);

#endif