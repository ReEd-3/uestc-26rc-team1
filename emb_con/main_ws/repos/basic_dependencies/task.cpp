#include "task.hpp"

Task::Add_Subtask(std::shared_ptr<Task> sub_task) 
{
    sub_tasks_.pushback(sub_task);
}

Task::Update() 
{
    if (children_.empty() || status_ != TaskStatus_t::RUNNING) {
        // 叶子任务默认无操作，必须由子类重写
        return;
    }

    // 更新所有子任务
    bool allFinished = true;
    for (auto& st : sub_tasks_) {
        st->Update();
        switch (st->Get_Status()) {
            case TaskStatus_t::FINISHED:
                break;
            // 出现错误直接断掉整条任务树
            case TaskStatus_t::ERROR:
                status_ = TaskStatus_t::ERROR;
                return;
            default:
            allFinished = false;
        }
    }

    // 全部任务完成之后进入结束状态
    if (allFinished) {
        Set_Status(TaskState::FINISHED);
    }
}

Task::Get_Handle() 
{
    return handle_;
}

Task::Get_Status()
{
    return status_;
}

Task::Set_Handle(void* handle) 
{
    handle_ = handle;
}

Task::Set_Status(TaskStatus_t new_status) {
    status_ = new_status;
}