#ifndef TASK_HPP
#define TASK_HPP

#include <memory>
#include <stdint.h>

// 强枚举表示任务状态
enum class TaskStatus_t : uint8_t{
    IDLE,
    RUNNING,
    PAUSED,
    FINISHED,
    ERROR,
}

// 一个可以用于递归增加子任务的任务类型
class Task {
public:
    explicit Task() : status_(TaskStatus_t::IDLE) {}  // 初始化为空闲状态
    ~Task();

    // 添加子任务
    void Add_Subtask(std::shared_ptr<Task> sub_task);

    // 更新函数
    virtual void Update();

    // 操作任务状态
    TaskStatus_t const Get_Status();
    void Set_Status(TaskStatus_t new_status);

    // 操作句柄
    void* const Get_Handle();
    void Set_Handle(void* handle);

protected:
    // 任务状态
    TaskStatus_t status_;

    // 任务句柄
    void* handle_;

    // 子任务容器
    std::vector<shared_ptr<Task>> sub_tasks_;
}

#endif