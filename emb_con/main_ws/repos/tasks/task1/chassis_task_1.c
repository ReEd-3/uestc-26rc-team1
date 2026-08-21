#include "chassis_task_1.h"
#include "cmsis_os2.h"
// #include <math.h>  // 精确校准/巡线旧代码暂时不用，注释保留

#ifndef TASK1_PI
#define TASK1_PI 3.14159265358979323846
#endif


extern osSemaphoreId_t Task1_ArmGetHandle;

extern volatile uint8_t arm_flag1;


/* 任务1初始化 */
void Chassis_Task1_Init(Chassis_Task1 *t1, Chassis *chassis)
{
    if (t1 == NULL || chassis == NULL) {
        return;
    }

    t1->chassis = chassis;
    t1->state = START;
    t1->host_event = CHASSIS_TASK1_HOST_EVENT_NONE;
    t1->task_done_sent = 0;

    /* TODO: 按实际场地填写第一、二段移动量 */
    t1->move1_x = 2.6;
    t1->move1_y = 0.6;
    t1->move2_x = 0.5;
    t1->move2_y = -0.5;
    t1->move3_x = -3.0;
    t1->move3_y = 2.0;
}

/* 设置第一段相对位移：x 为前进方向，y 为横移方向 */
void Chassis_Task1_SetMove1(Chassis_Task1 *t1, double x, double y)
{
    if (t1 == NULL) {
        return;
    }

    t1->move1_x = x;
    t1->move1_y = y;
}

/* 设置第二段前进距离 */
void Chassis_Task1_SetMove2(Chassis_Task1 *t1, double x, double y)
{
    if (t1 == NULL) {
        return;
    }

    t1->move2_x = x;
    t1->move2_y = y;
}

void Chassis_Task1_SetMove3(Chassis_Task1 *t1, double x, double y)
{
    if (t1 == NULL) {
        return;
    }

    t1->move3_x = x;
    t1->move3_y = y;
}

/* 任务1状态机更新 */
void Chassis_Task1_Update(Chassis_Task1 *t1)
{
    if (t1 == NULL || t1->chassis == NULL) {
        return;
    }

    switch (t1->state) {
        case START:
            /* 启动后直接开始第一段：先重置码盘坐标系，再移动指定 (x, y) */
            Chassis_ResetEncoderPose(t1->chassis);
            Chassis_MoveRelative(t1->chassis, t1->move1_x, t1->move1_y, 0.0);
            t1->state = MOVE_1;
            break;

        case MOVE_1:
            /* 第一段到达后上报，并立即连续切换第二段前进 */
            if (Chassis_Arrived(t1->chassis)) {
                // 1、2段连贯，不在中间停车
                // Chassis_Stop(t1->chassis);
                t1->host_event = CHASSIS_TASK1_HOST_EVENT_MOVE_DONE_1;
                /* 第二段开始前也重置码盘坐标系 */
                Chassis_ResetEncoderPose(t1->chassis);
                Chassis_MoveRelative(t1->chassis, t1->move2_x, t1->move2_y, 0.0);
                t1->state = MOVE_2;
            }
            break;

        case MOVE_2:
            /* 第二段到达后上报，并立即连续切换第三段前进 */
            if (Chassis_Arrived(t1->chassis)) {
                // 2、3段连贯，不在中间停车
                Chassis_Stop(t1->chassis);
                t1->host_event = CHASSIS_TASK1_HOST_EVENT_MOVE_DONE_2;
                osSemaphoreRelease(Task1_ArmGetHandle);
                t1->state = CUBE_GET_1;
            }
            break;

        case CUBE_GET_1:
            if (arm_flag1) {
                arm_flag1 = 0;
                // Chassis_Stop(t1->chassis);
                Chassis_ResetEncoderPose(t1->chassis);
                Chassis_MoveRelative(t1->chassis, t1->move3_x, t1->move3_y, 0.0);
                t1->host_event = CHASSIS_TASK1_HOST_EVENT_ARMGET_DONE;
                t1->state = MOVE_3;
            }
            break;
        case MOVE_3:
            /* 第三段到达后停车、上报，任务完成 */
            if (Chassis_Arrived(t1->chassis)) {
                Chassis_Stop(t1->chassis);
                t1->host_event = CHASSIS_TASK1_HOST_EVENT_MOVE_DONE_3;
                t1->state = TASK1_DONE;
            }
            break;

        case TASK1_DONE:
            /* 等 MOVE_DONE_2 被 Poll 取走后，再补发 TASK_DONE */
            if (!t1->task_done_sent && t1->host_event == CHASSIS_TASK1_HOST_EVENT_NONE) {
                t1->host_event = CHASSIS_TASK1_HOST_EVENT_TASK_DONE;
                t1->task_done_sent = 1;
            }
            Chassis_Stop(t1->chassis);
            break;

        default:
            break;
    }

    /* 每个控制周期都更新底盘 */
    Chassis_Update(t1->chassis);
}

// 获取任务事件状态，获取后事件状态置NONE
Chassis_Task1_HostEvent Chassis_Task1_PopHostEvent(Chassis_Task1 *t1)
{
    if (t1 == NULL) {
        return CHASSIS_TASK1_HOST_EVENT_NONE;
    }

    Chassis_Task1_HostEvent event = t1->host_event;
    t1->host_event = CHASSIS_TASK1_HOST_EVENT_NONE;
    return event;
}
