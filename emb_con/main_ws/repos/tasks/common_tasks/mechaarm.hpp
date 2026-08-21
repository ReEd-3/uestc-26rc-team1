#ifndef MECHAARM_GET_TASK_HPP
#define MECHAARM_GET_TASK_HPP

#include "task_tree.hpp"
#include "stm32h7xx_hal.h"

class MechaArmGetTask : public Task {
public:
    explicit MechaArmGetTask(uint32_t placeholder_ms = 2000)
        : placeholder_ms_(placeholder_ms) {}

    void Start() override
    {
        Task::Start();
        start_tick_ = 0;
    }

    void Update() override
    {
        if (status_ != TaskExeStatus::RUNNING) {
            return;
        }

        if (start_tick_ == 0) {
            start_tick_ = HAL_GetTick();
        }

        if (HAL_GetTick() - start_tick_ >= placeholder_ms_) {
            Set_Status(TaskExeStatus::FINISHED);
        }
    }

private:
    uint32_t placeholder_ms_;
    uint32_t start_tick_ = 0;
};

#endif
