#ifndef BEZIERTRAJPLAN_H
#define BEZIERTRAJPLAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define BEZIER_MAX_RESOLUTION 1000  // 最大查表数量

#define GAUSS_POINT1 -0.8611363116
#define GAUSS_WEIGHT1 0.3478548451
#define GAUSS_POINT2 -0.3399810436
#define GAUSS_WEIGHT2 0.6521451549
#define GAUSS_POINT3 0.3399810436
#define GAUSS_WEIGHT3 0.6521451549
#define GAUSS_POINT4 0.8611363116
#define GAUSS_WEIGHT4 0.3478548451

typedef struct PNT {
    double x;
    double y;
} _2D_Point;

typedef struct SB {
    _2D_Point p0, p1, p2;  // 贝塞尔曲线控制点
    double lst_integral;  // 上一时刻路程积分
    double lst_t;
    double max_err;  // 允许的最大误差
} SquareBezier;

typedef struct CB {
    _2D_Point p0, p1, p2, p3;  // 贝塞尔曲线控制点
    double max_err;  // 允许的最大误差
} CubeBezier;

// 二次贝塞尔曲线计算
_2D_Point SquareBezier_Compute(SquareBezier *sb, double t);

// 三次贝塞尔曲线计算
_2D_Point CubeBezier_Compute(CubeBezier *sb, double t);

// 计算贝塞尔曲线速度大小（标量）
double SquareBezier_Differential(SquareBezier *sb, double t);
double CubeBezier_Differential(CubeBezier *sb, double t);

// 计算贝塞尔曲线速度向量（用于 vx/vy）
_2D_Point SquareBezier_Derivative(SquareBezier *sb, double t);
_2D_Point CubeBezier_Derivative(CubeBezier *sb, double t);

// 高斯-勒让德求二次/三次贝塞尔曲线弧长积分
double Gauss_Segment_arc_Square(SquareBezier *sb, double a, double b);
double Gauss_Segment_arc_Cube(CubeBezier *bz, double a, double b);

// 二分搜索+线性插值：由目标弧长反求贝塞尔参数 t
double SquareBezier_ArcToParam(SquareBezier *sb, double target_arc, double a, double b);
double CubeBezier_ArcToParam(CubeBezier *bz, double target_arc, double a, double b);

#ifdef __cplusplus
}
#endif

#endif
