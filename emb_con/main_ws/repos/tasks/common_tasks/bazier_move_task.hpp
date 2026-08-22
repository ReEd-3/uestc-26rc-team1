#ifndef BEZIER_MOVE_TASK_HPP
#define BEZIER_MOVE_TASK_HPP

#include "task_tree.hpp"

extern "C" {
#include "chassis.h"
#include "beziertrajplan.h"
#include "cubictrajplan.h"
}

#include <math.h>

class SquareBezierMoveTask : public Task {
public:
    SquareBezierMoveTask(Chassis* chassis,
                //    _2D_Point p0,
                   _2D_Point p1,
                   _2D_Point p2,
                   double v0,
                   double vf,
                   double duration,
                   double max_err
                )
        : chassis_(chassis),
        //   p0_(p0),
          p1_(p1),
          p2_(p2),
          v0_(v0),
          vf_(vf),
          duration_(duration),
          max_err_(max_err) {}

    void Start() override
    {
        Task::Start();

        // 1. 设置二次贝塞尔控制点
        _2D_Point start_p = {chassis_->eo->abs_x, chassis_->eo->abs_y};
        sb_.p0 = start_p;
        sb_.p1 = p1_;
        sb_.p2 = p2_;

        sb_.p1.x = start_p.x + p1_.x;
        sb_.p1.y = start_p.y + p1_.y;

        sb_.p2.x = start_p.x + p2_.x;
        sb_.p2.y = start_p.y + p2_.y;

        sb_.max_err = max_err_;

        // 2. 计算总弧长
        total_arc_ = Gauss_Segment_arc_Square(&sb_, 0.0, 1.0);

        // 3. 三次多项式规划：弧长 0 -> total_arc，初末速度 v0/vf
        Cubic1D_Plan(&speed_,
                     0.0, total_arc_,
                     v0_, vf_,
                     duration_);

        start_tick_ = HAL_GetTick();
    }

    void Update() override
    {
        if (status_ != TaskExeStatus::RUNNING) {
            return;
        }

        double t = (HAL_GetTick() - start_tick_) / 1000.0;

        // 1. 由时间得到目标弧长
        speed_.cur_t = t;
        double s_target = Cubic1D_Sample(&speed_);

        if (s_target >= total_arc_) {
            // 位置PID：停到终点
            Chassis_FollowTarget(chassis_, sb_.p2.x, sb_.p2.y, chassis_->mn.yaw);
            Set_Status(TaskExeStatus::FINISHED);
            return;
        }

        // 2. 由弧长反求贝塞尔参数 u
        double u = SquareBezier_ArcToParam(&sb_, s_target, 0.0, 1.0);

        // 3. 得到目标位置
        _2D_Point pos = SquareBezier_Compute(&sb_, u);

        // 4. 位置PID：只更新绝对目标，不重置PID
        Chassis_FollowTarget(chassis_, pos.x, pos.y, chassis_->mn.yaw);

        // ---- 旧速度PID逻辑保留注释 ----
        // _2D_Point d = SquareBezier_Derivative(&sb_, u);
        // double v_s = Cubic1D_Velocity(&speed_);
        // double ds_du = hypot(d.x, d.y);
        // double vx_w = d.x * v_s / ds_du;
        // double vy_w = d.y * v_s / ds_du;
        // double yaw = chassis_->mn.yaw;
        // double vx = vx_w * cos(yaw) + vy_w * sin(yaw);
        // double vy = -vx_w * sin(yaw) + vy_w * cos(yaw);
        // Chassis_SetVelocity(chassis_, vx, vy, 0.0);
    }

private:
    // 底盘句柄
    Chassis* chassis_;

    // 贝塞尔控制点
    _2D_Point p0_;
    _2D_Point p1_;
    _2D_Point p2_;

    // 初末速度以及最大容错
    double v0_;
    double vf_;
    double duration_;
    double max_err_;
    
    SquareBezier sb_;
    CubicSegment1D speed_;
    double total_arc_ = 0.0;
    uint32_t start_tick_ = 0;
};

#endif