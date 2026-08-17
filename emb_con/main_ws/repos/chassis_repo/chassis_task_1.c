#include "chassis_task_1.h"
#include <math.h>

#ifndef TASK1_PI
#define TASK1_PI 3.14159265358979323846
#endif

/* 右转 90°，正负号以实际 yaw 方向为准 */
#define TASK1_TURN_ANGLE (-TASK1_PI / 2.0)

void Chassis_Task1_Init(Chassis_Task1 *t1, Chassis *chassis)
{
    if (t1 == NULL || chassis == NULL) {
        return;
    }

    t1->chassis = chassis;
    t1->state = START;
    t1->host_event = CHASSIS_TASK1_HOST_EVENT_NONE;  // 

    t1->line_center = 0.0;
    t1->line_slope = 0.0;
    t1->tower_distance = 0.0;
    t1->target_distance = 0.3;

    t1->line_found = 0;
    t1->junction_signal = 0;
}

void Chassis_Task1_SetTargetDistance(Chassis_Task1 *t1, double target_distance)
{
    if (t1 == NULL) {
        return;
    }

    t1->target_distance = target_distance;
}

void Chassis_Task1_OnLineData(Chassis_Task1 *t1, double center, double slope)
{
    if (t1 == NULL) {
        return;
    }

    t1->line_center = center;
    t1->line_slope = slope;
    t1->line_found = 1;
}

void Chassis_Task1_OnJunctionSignal(Chassis_Task1 *t1)
{
    if (t1 == NULL) {
        return;
    }

    t1->junction_signal = 1;
}

void Chassis_Task1_OnTowerDistance(Chassis_Task1 *t1, double distance)
{
    if (t1 == NULL) {
        return;
    }

    t1->tower_distance = distance;
}

void Chassis_Task1_Update(Chassis_Task1 *t1)
{
    if (t1 == NULL || t1->chassis == NULL) {
        return;
    }

    switch (t1->state) {
        case START:
            /* 等待第一帧巡线数据 */
            if (t1->line_found) {
                t1->line_found = 0;
                t1->state = FIND_LINE_1;
            }
            break;

        case FIND_LINE_1:
            /* 找到线后进入第一段巡线 */
            if (t1->line_found) {
                t1->line_found = 0;
                t1->state = FOLLOW_LINE_1;
            }
            break;

        case FOLLOW_LINE_1:
            /* TODO: 用 line_center/line_slope 做巡线控制
             * Chassis_SetVelocity(chassis, vx, vy, omega);
             */
            if (t1->junction_signal) {
                t1->junction_signal = 0;
                Chassis_Stop(t1->chassis);
                Chassis_MoveRelative(t1->chassis, 0.0, 0.0, TASK1_TURN_ANGLE);
                t1->state = TURN_RIGHT_1;
            }
            break;

        case TURN_RIGHT_1:
            /* 第一次右转完成，通知上位机 */
            if (Chassis_Arrived(t1->chassis)) {
                Chassis_Stop(t1->chassis);
                t1->host_event = CHASSIS_TASK1_HOST_EVENT_TURN_DONE_1;
                t1->state = FIND_LINE_2;
            }
            break;

        case FIND_LINE_2:
            /* 右转后等待上位机重新发巡线数据 */
            if (t1->line_found) {
                t1->line_found = 0;
                t1->state = FOLLOW_LINE_2;
            }
            break;

        case FOLLOW_LINE_2:
            /* TODO: 第二段巡线控制，和第一段相同 */
            if (t1->junction_signal) {
                t1->junction_signal = 0;
                Chassis_Stop(t1->chassis);
                Chassis_MoveRelative(t1->chassis, 0.0, 0.0, TASK1_TURN_ANGLE);
                t1->state = TURN_RIGHT_2;
            }
            break;

        case TURN_RIGHT_2:
            /* 第二次右转完成，通知上位机 */
            if (Chassis_Arrived(t1->chassis)) {
                Chassis_Stop(t1->chassis);
                t1->host_event = CHASSIS_TASK1_HOST_EVENT_TURN_DONE_2;
                t1->state = FIND_LINE_3;
            }
            break;

        case FIND_LINE_3:
            /* 第二次右转后，等待面向塔的巡线/距离数据 */
            if (t1->line_found) {
                t1->line_found = 0;
                t1->state = TOWER_APPROACH;
            }
            break;

        case TOWER_APPROACH:
            /* TODO: 同时使用 tower_distance 和巡线数据控制
             * vx 来自距离误差，vy/omega 来自巡线
             * Chassis_SetVelocity(chassis, vx, vy, omega);
             */
            if (t1->tower_distance > 0.0 &&
                t1->tower_distance <= t1->target_distance) {
                Chassis_Stop(t1->chassis);
                t1->host_event = CHASSIS_TASK1_HOST_EVENT_TASK_DONE;
                t1->state = TASK1_DONE;
            }
            break;

        case TASK1_DONE:
        default:
            break;
    }

    /* 每个控制周期都更新底盘 */
    Chassis_Update(t1->chassis);
}

Chassis_Task1_CheckPoint Chassis_Task1_GetState(const Chassis_Task1 *t1)
{
    if (t1 == NULL) {
        return START;
    }

    return t1->state;
}

Chassis_Task1_HostEvent Chassis_Task1_PopHostEvent(Chassis_Task1 *t1)
{
    if (t1 == NULL) {
        return CHASSIS_TASK1_HOST_EVENT_NONE;
    }

    Chassis_Task1_HostEvent event = t1->host_event;
    t1->host_event = CHASSIS_TASK1_HOST_EVENT_NONE;
    return event;
}

uint8_t Chassis_Task1_IsDone(const Chassis_Task1 *t1)
{
    if (t1 == NULL) {
        return 0;
    }

    return (t1->state == TASK1_DONE) ? 1 : 0;
}
