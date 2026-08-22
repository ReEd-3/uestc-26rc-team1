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
            Chassis_FollowTarget(chassis_, sb_.p2.x, sb_.p2.y, chassis_->yis->yaw);
            Set_Status(TaskExeStatus::FINISHED);
            return;
        }

        // 2. 由弧长反求贝塞尔参数 u
        double u = SquareBezier_ArcToParam(&sb_, s_target, 0.0, 1.0);

        // 3. 得到目标位置
        _2D_Point pos = SquareBezier_Compute(&sb_, u);

        // 4. 位置PID：只更新绝对目标，不重置PID
        Chassis_FollowTarget(chassis_, pos.x, pos.y, chassis_->yis->yaw);
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

class CubeBezierMoveTask : public Task {
public:
    CubeBezierMoveTask(Chassis* chassis,
                //    _2D_Point p0,
                   _2D_Point p1,
                   _2D_Point p2,
                   _2D_Point p3,
                //    double v0,
                   double vf,
                   double duration,
                   double max_err
                )
        : chassis_(chassis),
        //   p0_(p0),
          p1_(p1),
          p2_(p2),
          p3_(p3),
        //   v0_(v0),
          vf_(vf),
          duration_(duration),
          max_err_(max_err) {}

    void Start() override
    {
        Task::Start();

        // 1. 设置三次贝塞尔控制点
        _2D_Point start_p = {chassis_->eo->abs_x, chassis_->eo->abs_y};
        cb_.p0 = start_p;
        cb_.p1 = p1_;
        cb_.p2 = p2_;
        cb_.p3 = p3_;

        cb_.p1.x = start_p.x + p1_.x;
        cb_.p1.y = start_p.y + p1_.y;

        cb_.p2.x = start_p.x + p2_.x;
        cb_.p2.y = start_p.y + p2_.y;

        cb_.p3.x = start_p.x + p3_.x;
        cb_.p3.y = start_p.y + p3_.y;

        cb_.max_err = max_err_;

        // 2. 计算总弧长
        total_arc_ = Gauss_Segment_arc_Cube(&cb_, 0.0, 1.0);

        // 3. 三次多项式规划：弧长 0 -> total_arc，初末速度 v0/vf
        v0_ = sqrt(pow(chassis_->eo->abs_vx, 2) + pow(chassis_->eo->abs_vy, 2));
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
            Chassis_FollowTarget(chassis_, cb_.p2.x, cb_.p2.y, chassis_->yis->yaw);
            Set_Status(TaskExeStatus::FINISHED);
            return;
        }

        // 2. 由弧长反求贝塞尔参数 u
        double u = CubeBezier_ArcToParam(&cb_, s_target, 0.0, 1.0);

        // 3. 得到目标位置
        _2D_Point pos = CubeBezier_Compute(&cb_, u);

        // 4. 位置PID：只更新绝对目标，不重置PID
        Chassis_FollowTarget(chassis_, pos.x, pos.y, chassis_->yis->yaw);
    }

private:
    // 底盘句柄
    Chassis* chassis_;

    // 贝塞尔控制点
    _2D_Point p0_;
    _2D_Point p1_;
    _2D_Point p2_;
    _2D_Point p3_;

    // 初末速度以及最大容错
    double v0_;
    double vf_;
    double duration_;
    double max_err_;
    
    CubeBezier cb_;
    CubicSegment1D speed_;
    double total_arc_ = 0.0;
    uint32_t start_tick_ = 0;
};

#endif