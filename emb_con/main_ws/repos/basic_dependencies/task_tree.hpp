#ifndef TASK_HPP
#define TASK_HPP

#include <memory>
#include <utility>
#include <cstdint>
#include <cstddef>
#include <vector>

// 强枚举表示任务状态
enum class TaskExeStatus : uint8_t{
    IDLE,
    RUNNING,
    PAUSED,
    FINISHED,
    ERROR,
};

// 一个可以用于递归增加子任务的任务类型
class Task {
public:
    explicit Task() 
        : status_(TaskExeStatus::IDLE),
        current_(0) {}  // 初始化为空闲状态
    virtual ~Task() = default;

    // 添加子任务
    void Add_Subtask(std::shared_ptr<Task> sub_task)
    {
        sub_tasks_.push_back(std::move(sub_task));  // 移动共享指针
    }

    // 任务启动或者复位
    virtual void Start()
    {
        current_ = 0;
        status_ = TaskExeStatus::RUNNING;

        // 递归复位所有子任务
        for (auto& sub : sub_tasks_) {
            sub->Start();
        }
    }

    // 更新函数
    virtual void Update()
    {
        if (status_ != TaskExeStatus::RUNNING) {
            // 叶子任务默认无操作，必须由子类重写
            return;
        }

        if (current_ >= sub_tasks_.size()) {
            status_ = TaskExeStatus::FINISHED;
            return;
        }

        // 对当前任务进行处理
        auto& cur = sub_tasks_[current_];
        // 启动任务
        if (cur->Get_Status() == TaskExeStatus::IDLE) {
            cur->Start();
        }
        // 执行任务
        cur->Update();

        // 检测任务是否结束或者是否出错
        if (cur->Get_Status() == TaskExeStatus::FINISHED) {
            ++current_;
        } 
        else if (cur->Get_Status() == TaskExeStatus::ERROR) {
            status_ = TaskExeStatus::ERROR;
        }
    }

    // 操作任务状态
    TaskExeStatus const Get_Status() { return status_; }
    void Set_Status(TaskExeStatus new_status) { status_ = new_status; }

protected:
    // 任务状态
    TaskExeStatus status_;

    // 当前任务索引
    std::size_t current_ = 0;

    // 子任务容器
    std::vector<std::shared_ptr<Task>> sub_tasks_;
};

#endif