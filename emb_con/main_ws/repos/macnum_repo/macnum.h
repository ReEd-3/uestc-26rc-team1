#ifndef MACNUM_H
#define MACNUM_H

typedef struct {
    double wheel_radius;  // 轮子半径
    double half_wheelbase;  // 轴距的一半
    double half_track;  // 轮距的一半
    double tar_vx, tar_vy, tar_omega;  // 目标速度,转角
    double tar_rpm[4];  // 分别是FL, FR, RL, RR的rpm
    double rea_vx, rea_vy, rea_omega;  // 真实的运动状态
} Macnum;  // 麦轮解算句柄，专用来解算

void Macnum_Init (Macnum *mn, double wheel_radius, double half_wheelbase, double half_track);
void Macnum_SetTarget (Macnum *mn, double vx, double vy, double omega);
void Macnum_StateUpdate (Macnum *mn, double *rea_rpm);

#endif