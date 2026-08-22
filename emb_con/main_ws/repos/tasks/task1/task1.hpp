#ifndef CHASSIS_MOVE1_HPP
#define CHASSIS_MOVE1_HPP

#include "task_tree.hpp"
#include "mechaarm.hpp"
// #include "chassis_move.hpp"   // 旧逻辑保留注释
#include "bazier_move_task.hpp"

class Task1 : public Task {
public:
    explicit Task1(Chassis* chassis)
        : chassis_(chassis)
    {
        // 旧逻辑：使用 Chassis_Move
        // Add_Subtask(std::make_shared<Chassis_Move>(chassis, 2.6, 0.6));   // chassis move 1
        // Add_Subtask(std::make_shared<Chassis_Move>(chassis, 0.5, -0.5));  // chassis move 2
        // Add_Subtask(std::make_shared<MechaArmGetTask>(2000));                // mechaarm get 1
        // Add_Subtask(std::make_shared<Chassis_Move>(chassis, -3.0, 2.0));  // chassis move 3

        // 新逻辑：使用贝塞尔 + 三次多项式速度规划（参数待你调整）
        Add_Subtask(std::make_shared<SquareBezierMoveTask>(
            chassis,
            _2D_Point{1.5, 1.5},   // p1 相对偏移，待调整
            _2D_Point{3.2, 0.1},   // p2 相对终点，待调整
            0.0,                   // v0
            0.0,                   // vf
            4.0,                    // duration
            0.02                    // max_err
        ));

        Add_Subtask(std::make_shared<SquareBezierMoveTask>(
            chassis,
            _2D_Point{1.5, 1.5},   // p1 相对偏移，待调整
            _2D_Point{3.2, 0.1},   // p2 相对终点，待调整
            0.0,                   // v0
            0.0,                   // vf
            4.0,                    // duration
            0.02                    // max_err
        ));

        // Add_Subtask(std::make_shared<SquareBezierMoveTask>(
        //     chassis,
        //     _2D_Point{0.2, -0.2},
        //     _2D_Point{0.5, -0.5},
        //     0.0, 0.0, 1.5
        // ));

        // Add_Subtask(std::make_shared<MechaArmGetTask>(2000));

        // Add_Subtask(std::make_shared<SquareBezierMoveTask>(
        //     chassis,
        //     _2D_Point{-1.0, 1.0},
        //     _2D_Point{-3.0, 2.0},
        //     0.0, 0.0, 4.0
        // ));
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
