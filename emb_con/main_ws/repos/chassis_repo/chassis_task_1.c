#include "chassis_task_1.h"
// #include <math.h>  // 精确校准/巡线旧代码暂时不用，注释保留

#ifndef TASK1_PI
#define TASK1_PI 3.14159265358979323846
#endif

/* 右转 90°，正负号以实际 yaw 方向为准（旧巡线方案，注释保留） */
// #define TASK1_TURN_ANGLE (-TASK1_PI / 2.0)

/* 第三段微调参数：精确校准暂时注释停用，代码保留 */
// #define TASK1_APPROACH_KP       0.3   /* 距离误差 -> 前进速度 */
// #define TASK1_APPROACH_DEADBAND 0.01  /* 距离误差小于该值认为到位，单位与 tower_distance 相同 */

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
    t1->move1_x = 0.0;
    t1->move1_y = 0.0;
    t1->move2_distance = 0.0;

    // t1->line_center = 0.0;   // 旧巡线方案，注释保留
    // t1->line_slope = 0.0;    // 旧巡线方案，注释保留
    // t1->tower_distance = 0.0;   // 精确校准，注释保留
    // t1->target_distance = 0.4;  // 精确校准，注释保留

    // t1->line_found = 0;      // 旧巡线方案，注释保留
    // t1->junction_signal = 0; // 旧巡线方案，注释保留
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
void Chassis_Task1_SetMove2Distance(Chassis_Task1 *t1, double distance)
{
    if (t1 == NULL) {
        return;
    }

    t1->move2_distance = distance;
}

// /* 设置目标停车距离（精确校准，注释保留） */
// void Chassis_Task1_SetTargetDistance(Chassis_Task1 *t1, double target_distance)
// {
//     if (t1 == NULL) {
//         return;
//     }

//     t1->target_distance = target_distance;
// }

// /* 旧巡线方案，注释保留 */
// void Chassis_Task1_OnLineData(Chassis_Task1 *t1, double center, double slope)
// {
//     if (t1 == NULL) {
//         return;
//     }

//     t1->line_center = center;
//     t1->line_slope = slope;
//     t1->line_found = 1;
// }

// /* 旧巡线方案，注释保留 */
// void Chassis_Task1_OnJunctionSignal(Chassis_Task1 *t1)
// {
//     if (t1 == NULL) {
//         return;
//     }

//     t1->junction_signal = 1;
// }

// /* 接收深度摄像头到塔距离（精确校准，注释保留） */
// void Chassis_Task1_OnTowerDistance(Chassis_Task1 *t1, double distance)
// {
//     if (t1 == NULL) {
//         return;
//     }

//     t1->tower_distance = distance;
// }

/* 任务1状态机更新 */
void Chassis_Task1_Update(Chassis_Task1 *t1)
{
    if (t1 == NULL || t1->chassis == NULL) {
        return;
    }

    switch (t1->state) {
        case START:
            /* 启动后直接开始第一段：向右前方移动指定 (x, y) */
            Chassis_MoveRelative(t1->chassis, t1->move1_x, t1->move1_y, 0.0);
            t1->state = MOVE_1;
            break;

        case MOVE_1:
            /* 第一段到达后停车、上报，并立即下发第二段前进 */
            if (Chassis_Arrived(t1->chassis)) {
                Chassis_Stop(t1->chassis);
                t1->host_event = CHASSIS_TASK1_HOST_EVENT_MOVE_DONE_1;
                Chassis_MoveRelative(t1->chassis, t1->move2_distance, 0.0, 0.0);
                t1->state = MOVE_2;
            }
            break;

        case MOVE_2:
            /* 第二段到达后停车、上报，任务完成 */
            if (Chassis_Arrived(t1->chassis)) {
                Chassis_Stop(t1->chassis);
                t1->host_event = CHASSIS_TASK1_HOST_EVENT_MOVE_DONE_2;
                t1->state = TASK1_DONE;
            }
            break;

        // case FIND_LINE_1:
        //     /* 找到线后进入第一段巡线（旧巡线方案，注释保留） */
        //     if (t1->line_found) {
        //         t1->line_found = 0;
        //         t1->state = FOLLOW_LINE_1;
        //     }
        //     break;

        // case FOLLOW_LINE_1:
        //     /* TODO: 用 line_center/line_slope 做巡线控制
        //      * Chassis_SetVelocity(chassis, vx, vy, omega);
        //      */
        //     if (t1->junction_signal) {
        //         t1->junction_signal = 0;
        //         Chassis_Stop(t1->chassis);
        //         Chassis_MoveRelative(t1->chassis, 0.0, 0.0, TASK1_TURN_ANGLE);
        //         t1->state = TURN_RIGHT_1;
        //     }
        //     break;

        // case TURN_RIGHT_1:
        //     /* 第一次右转完成，通知上位机 */
        //     if (Chassis_Arrived(t1->chassis)) {
        //         Chassis_Stop(t1->chassis);
        //         t1->host_event = CHASSIS_TASK1_HOST_EVENT_TURN_DONE_1;
        //         t1->state = FIND_LINE_2;
        //     }
        //     break;

        // case FIND_LINE_2:
        //     /* 右转后等待上位机重新发巡线数据 */
        //     if (t1->line_found) {
        //         t1->line_found = 0;
        //         t1->state = FOLLOW_LINE_2;
        //     }
        //     break;

        // case FOLLOW_LINE_2:
        //     /* TODO: 第二段巡线控制，和第一段相同 */
        //     if (t1->junction_signal) {
        //         t1->junction_signal = 0;
        //         Chassis_Stop(t1->chassis);
        //         Chassis_MoveRelative(t1->chassis, 0.0, 0.0, TASK1_TURN_ANGLE);
        //         t1->state = TURN_RIGHT_2;
        //     }
        //     break;

        // case TURN_RIGHT_2:
        //     /* 第二次右转完成，通知上位机 */
        //     if (Chassis_Arrived(t1->chassis)) {
        //         Chassis_Stop(t1->chassis);
        //         t1->host_event = CHASSIS_TASK1_HOST_EVENT_TURN_DONE_2;
        //         t1->state = FIND_LINE_3;
        //     }
        //     break;

        // case FIND_LINE_3:
        //     /* 第二次右转后，等待面向塔的巡线/距离数据 */
        //     if (t1->line_found) {
        //         t1->line_found = 0;
        //         t1->state = TOWER_APPROACH;
        //     }
        //     break;

        // case TOWER_APPROACH:
        //     /* 根据 tower_distance 做闭环微调；无有效距离时先停车等待（精确校准，注释保留） */
        //     if (t1->tower_distance <= 0.0) {
        //         Chassis_Stop(t1->chassis);
        //         break;
        //     }

        //     {
        //         double err = t1->tower_distance - t1->target_distance;
        //         if (fabs(err) <= TASK1_APPROACH_DEADBAND) {
        //             Chassis_Stop(t1->chassis);
        //             t1->host_event = CHASSIS_TASK1_HOST_EVENT_TASK_DONE;
        //             t1->state = TASK1_DONE;
        //         } else {
        //             double vx = err * TASK1_APPROACH_KP;
        //             double max_vx = t1->chassis->max_vx;
        //             if (vx > max_vx) {
        //                 vx = max_vx;
        //             } else if (vx < -max_vx) {
        //                 vx = -max_vx;
        //             }
        //             Chassis_SetVelocity(t1->chassis, vx, 0.0, 0.0);
        //         }
        //     }
        //     break;

        case TASK1_DONE:
            /* 等 MOVE_DONE_2 被 Poll 取走后，再补发 TASK_DONE */
            if (!t1->task_done_sent && t1->host_event == CHASSIS_TASK1_HOST_EVENT_NONE) {
                t1->host_event = CHASSIS_TASK1_HOST_EVENT_TASK_DONE;
                t1->task_done_sent = 1;
            }
            break;

        default:
            break;
    }

    /* 每个控制周期都更新底盘 */
    Chassis_Update(t1->chassis);
}

// Chassis_Task1_CheckPoint Chassis_Task1_GetState(const Chassis_Task1 *t1)
// {
//     if (t1 == NULL) {
//         return START;
//     }

//     return t1->state;
// }

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

// 检测任务1是否完成
// uint8_t Chassis_Task1_IsDone(const Chassis_Task1 *t1)
// {
//     if (t1 == NULL) {
//         return 0;
//     }

//     return (t1->state == TASK1_DONE) ? 1 : 0;
// }
