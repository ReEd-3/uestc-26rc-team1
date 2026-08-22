#ifndef CUBICTRAJPLAN_H
#define CUBICTRAJPLAN_H

#ifdef __cplusplus
extern "C" {
#endif

// y(x) = a3*x^3 + a2*x^2 + a1*x + a0;
typedef struct C1D {
    double a0, a1, a2, a3;  // 多项式系数
    double T;  // 时长
    double x0, x1;  // 初末位置
    double v0, v1;  // 初末速度
    double cur_pos;  // 当前位置
    double cur_t;  // 当前时间
} CubicSegment1D;

// 计算三次多项式参数
void Cubic1D_Plan(CubicSegment1D *seg, 
    double x0, double x1,
    double v0, double v1,
    double T
);

// 计算当前时间对应位移值，速度值
double Cubic1D_Sample(const CubicSegment1D *seg);
double Cubic1D_Velocity(const CubicSegment1D *seg);

#ifdef __cplusplus
}
#endif

#endif