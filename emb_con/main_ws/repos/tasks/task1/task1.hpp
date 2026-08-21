#ifndef CHASSIS_MOVE1_HPP
#define CHASSIS_MOVE1_HPP

#include "task_tree.hpp"
#include "chassis_move.hpp"
#include "mechaarm.hpp"

class Task1 : public Task {
public:
    explicit Task1(Chassis* chassis)
        : chassis_(chassis)
    {
        Add_Subtask(std::make_shared<Chassis_Move>(chassis, 2.6, 0.6));   // chassis move 1
        Add_Subtask(std::make_shared<Chassis_Move>(chassis, 0.5, -0.5));  // chassis move 2
        Add_Subtask(std::make_shared<MechaArmGetTask>(2000));                // mechaarm get 1
        Add_Subtask(std::make_shared<Chassis_Move>(chassis, -3.0, 2.0));  // chassis move 3
    }
    
    void Update() override
    {
        Task::Update();               // 顺序推进四个子任务
        Chassis_Update(chassis_);     // 每个控制周期驱动底盘
    }

private:
    Chassis* chassis_;
};

#endif