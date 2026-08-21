#ifndef CHASSIS_MOVE_HPP
#define CHASSIS_MOVE_HPP

#include "task_tree.hpp"

extern "C" {
#include "chassis.h"
}

class Chassis_Move : public Task {
public:
    Chassis_Move(Chassis* chassis, double dx, double dy, double dyaw = 0.0)
        : chassis_(chassis), dx_(dx), dy_(dy), dyaw_(dyaw) {}
    
    // 虚函数重写
    void Start() override
    {
        Task::Start();
        started_ = false;
    }

    void Update() override
    {
        if (status_ != TaskExeStatus::RUNNING) {
            return;
        }

        if (!started_) {
            Chassis_ResetEncoderPose(chassis_);
            Chassis_MoveRelative(chassis_, dx_, dy_, dyaw_);
            started_ = true;
        }

        if (Chassis_Arrived(chassis_)) {
            Set_Status(TaskExeStatus::FINISHED);
        }
    }

private:
    // 任务操作句柄
    Chassis* chassis_;
    // 运动目标
    double dx_;
    double dy_;
    double dyaw_;
    // 开始标志位
    bool started_ = false;
};

#endif