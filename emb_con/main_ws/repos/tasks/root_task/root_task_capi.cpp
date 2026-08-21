#include "root_task.hpp"

static RootTask* g_root_task = nullptr;

extern "C" {

void RootTask_Init(Chassis* chassis)
{
    delete g_root_task;
    g_root_task = new RootTask(chassis);
    g_root_task->Start();
}

void RootTask_Update()
{
    if (g_root_task != nullptr) {
        g_root_task->Update();
    }
}

}