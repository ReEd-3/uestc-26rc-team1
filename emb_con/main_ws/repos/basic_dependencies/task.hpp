#ifndef TASK_HPP
#define TASK_HPP

class Task {
public:
    explicit Task (const std::string& name) {}
    ~Task();

    void add_subtask(std::unique_ptr<Task> sub_task);
private:

}

#endif