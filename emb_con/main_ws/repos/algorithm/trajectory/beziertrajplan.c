#include "beziertrajplan.h"
#include <math.h>

// 二次贝塞尔曲线计算
_2D_Point SquareBezier_Compute(SquareBezier *sb, double t) {
    double _t = 1 - t;
    _2D_Point ans;
    ans.x = _t * _t * sb->p0.x + 2.0 * _t * t * sb->p1.x + t * t * sb->p2.x;
    ans.y = _t * _t * sb->p0.y + 2.0 * _t * t * sb->p1.y + t * t * sb->p2.y;
    return ans;
}

// 三次贝塞尔曲线计算
_2D_Point CubeBezier_Compute(CubeBezier *bz, double t) {
    double _t = 1 - t;
    _2D_Point ans;
    ans.x = _t * _t * _t * bz->p0.x
          + 3.0 * _t * _t * t * bz->p1.x
          + 3.0 * t * t * _t * bz->p2.x
          + t * t * t * bz->p3.x;
    ans.y = _t * _t * _t * bz->p0.y
          + 3.0 * _t * _t * t * bz->p1.y
          + 3.0 * t * t * _t * bz->p2.y
          + t * t * t * bz->p3.y;
    return ans;
}

// 二次贝塞尔速度大小
double SquareBezier_Differential(SquareBezier *sb, double t) {
    double _t = 1 - t;
    double dx = 2.0 * _t * (sb->p1.x - sb->p0.x) + 2.0 * t * (sb->p2.x - sb->p1.x);
    double dy = 2.0 * _t * (sb->p1.y - sb->p0.y) + 2.0 * t * (sb->p2.y - sb->p1.y);
    return sqrt(dx * dx + dy * dy);
}

// 三次贝塞尔速度大小
double CubeBezier_Differential(CubeBezier *bz, double t) {
    double _t = 1 - t;
    double dx = 3.0 * _t * _t * (bz->p1.x - bz->p0.x)
              + 6.0 * _t * t * (bz->p2.x - bz->p1.x)
              + 3.0 * t * t * (bz->p3.x - bz->p2.x);
    double dy = 3.0 * _t * _t * (bz->p1.y - bz->p0.y)
              + 6.0 * _t * t * (bz->p2.y - bz->p1.y)
              + 3.0 * t * t * (bz->p3.y - bz->p2.y);
    return sqrt(dx * dx + dy * dy);
}

// 二次贝塞尔速度向量
_2D_Point SquareBezier_Derivative(SquareBezier *sb, double t) {
    double _t = 1 - t;
    _2D_Point d;
    d.x = 2.0 * _t * (sb->p1.x - sb->p0.x) + 2.0 * t * (sb->p2.x - sb->p1.x);
    d.y = 2.0 * _t * (sb->p1.y - sb->p0.y) + 2.0 * t * (sb->p2.y - sb->p1.y);
    return d;
}

// 三次贝塞尔速度向量
_2D_Point CubeBezier_Derivative(CubeBezier *bz, double t) {
    double _t = 1 - t;
    _2D_Point d;
    d.x = 3.0 * _t * _t * (bz->p1.x - bz->p0.x)
        + 6.0 * _t * t * (bz->p2.x - bz->p1.x)
        + 3.0 * t * t * (bz->p3.x - bz->p2.x);
    d.y = 3.0 * _t * _t * (bz->p1.y - bz->p0.y)
        + 6.0 * _t * t * (bz->p2.y - bz->p1.y)
        + 3.0 * t * t * (bz->p3.y - bz->p2.y);
    return d;
}

// 高斯-勒让德：二次贝塞尔弧长积分
double Gauss_Segment_arc_Square(SquareBezier *sb, double a, double b) {
    double mid = (a + b) * 0.5;
    double half = (b - a) * 0.5;

    double u1 = mid + half * GAUSS_POINT1;
    double u2 = mid + half * GAUSS_POINT2;
    double u3 = mid + half * GAUSS_POINT3;
    double u4 = mid + half * GAUSS_POINT4;

    double ans = GAUSS_WEIGHT1 * SquareBezier_Differential(sb, u1)
               + GAUSS_WEIGHT2 * SquareBezier_Differential(sb, u2)
               + GAUSS_WEIGHT3 * SquareBezier_Differential(sb, u3)
               + GAUSS_WEIGHT4 * SquareBezier_Differential(sb, u4);

    return ans * half;
}

// 高斯-勒让德：三次贝塞尔弧长积分
double Gauss_Segment_arc_Cube(CubeBezier *bz, double a, double b) {
    double mid = (a + b) * 0.5;
    double half = (b - a) * 0.5;

    double u1 = mid + half * GAUSS_POINT1;
    double u2 = mid + half * GAUSS_POINT2;
    double u3 = mid + half * GAUSS_POINT3;
    double u4 = mid + half * GAUSS_POINT4;

    double ans = GAUSS_WEIGHT1 * CubeBezier_Differential(bz, u1)
               + GAUSS_WEIGHT2 * CubeBezier_Differential(bz, u2)
               + GAUSS_WEIGHT3 * CubeBezier_Differential(bz, u3)
               + GAUSS_WEIGHT4 * CubeBezier_Differential(bz, u4);

    return ans * half;
}

// 迭代二分搜索 + 线性插值：二次贝塞尔由弧长反求参数 t
double SquareBezier_ArcToParam(SquareBezier *sb, double target_arc, double a, double b)
{
    double cur_a = a;
    double cur_b = b;

    double int_a = Gauss_Segment_arc_Square(sb, 0.0, cur_a);
    double int_b = Gauss_Segment_arc_Square(sb, 0.0, cur_b);

    for (int i = 0; i < 50; i++) {
        if (target_arc - int_a <= sb->max_err &&
            int_b - target_arc <= sb->max_err) {
            break;
        }

        double mid = (cur_a + cur_b) * 0.5;
        double int_mid = Gauss_Segment_arc_Square(sb, 0.0, mid);

        if (target_arc < int_mid) {
            cur_b = mid;
        } else {
            cur_a = mid;
        }

        int_a = Gauss_Segment_arc_Square(sb, 0.0, cur_a);
        int_b = Gauss_Segment_arc_Square(sb, 0.0, cur_b);
    }

    if (int_b <= int_a) {
        return cur_a;
    }

    return cur_a + (cur_b - cur_a) * (target_arc - int_a) / (int_b - int_a);
}


// 迭代二分搜索 + 线性插值：三次贝塞尔由弧长反求参数 t
double CubeBezier_ArcToParam(CubeBezier *bz, double target_arc, double a, double b)
{
    double cur_a = a;
    double cur_b = b;

    double int_a = Gauss_Segment_arc_Cube(bz, 0.0, cur_a);
    double int_b = Gauss_Segment_arc_Cube(bz, 0.0, cur_b);

    for (int i = 0; i < 50; i++) {
        if (target_arc - int_a <= bz->max_err &&
            int_b - target_arc <= bz->max_err) {
            break;
        }

        double mid = (cur_a + cur_b) * 0.5;
        double int_mid = Gauss_Segment_arc_Cube(bz, 0.0, mid);

        if (target_arc < int_mid) {
            cur_b = mid;
        } else {
            cur_a = mid;
        }

        int_a = Gauss_Segment_arc_Cube(bz, 0.0, cur_a);
        int_b = Gauss_Segment_arc_Cube(bz, 0.0, cur_b);
    }

    if (int_b <= int_a) {
        return cur_a;
    }

    return cur_a + (cur_b - cur_a) * (target_arc - int_a) / (int_b - int_a);
}
