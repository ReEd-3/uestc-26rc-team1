#include "cubictrajplan.h"
#include "beziertrajplan.h"

// 计算三次多项式参数
void Cubic1D_Plan(CubicSegment1D *seg, 
    double x0, double x1,
    double v0, double v1,
    double T
)
{
    double h = x1 - x0;
    seg->a0 = x0;
    seg->a1 = v0;
    seg->a2 = (3 * h) / (T * T) - (2 * v0 + v1) / T;
    seg->a3 = (-2 * h) / (T * T * T) + (v0 + v1) / T / T;
}

// 计算当前时间对应位移值
double Cubic1D_Sample(const CubicSegment1D *seg)
{
    return seg->a0 + 
    seg->a1 * seg->cur_t +
    seg->a2 * seg->cur_t * seg->cur_t +
    seg->a3 * seg->cur_t * seg->cur_t * seg->cur_t;
}

double Cubic1D_Velocity(const CubicSegment1D *seg) {
    double t = seg->cur_t;
    return seg->a1 + 2.0 * seg->a2 * t + 3.0 * seg->a3 * t * t;
}