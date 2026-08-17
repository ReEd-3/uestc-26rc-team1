#ifndef CHASSIS_TASK_1_H
#define CHASSIS_TASK_1_H

#include <stdint.h>
#include "chassis.h"

/* 任务1状态机检查点 */
typedef enum {
    START,  // 初始状态
    FIND_LINE_1,  // 先找到要巡的线
    FOLLOW_LINE_1,  // 第一段巡线，上位机发送右转信号的时候进入右转检查点
    TURN_RIGHT_1,  // 第一次右转，里程计到达目标后向上位机发信号并且进入寻找巡线的检查点
    FIND_LINE_2,  // 右转之后寻找巡线，收到上位机的信号之后再进入第二段巡线
    FOLLOW_LINE_2,  // 第二段巡线，上位机发送信号后再进入第二次右转
    TURN_RIGHT_2,  // 第二次右转，时机和第一次一样
    FIND_LINE_3,  // 第二次右转之后寻找巡线
    TOWER_APPROACH,  // 通过距离和巡线微调数据
    TASK1_DONE,  // 任务完成
} Chassis_Task1_CheckPoint;

/* 需要上报给上位机的事件 */
typedef enum {
    CHASSIS_TASK1_HOST_EVENT_NONE = 0,
    CHASSIS_TASK1_HOST_EVENT_TURN_DONE_1,  // 第一次右转完成
    CHASSIS_TASK1_HOST_EVENT_TURN_DONE_2,  // 第二次右转完成
    CHASSIS_TASK1_HOST_EVENT_TASK_DONE,    // 任务1完成
} Chassis_Task1_HostEvent;

/* 任务1句柄 */
typedef struct {
    Chassis *chassis;              // 指向底盘句柄
    Chassis_Task1_CheckPoint state; // 当前状态

    Chassis_Task1_HostEvent host_event; // 待上报给上位机的事件

    // 主机实时输入
    double line_center;    // 摄像头中线中心坐标
    double line_slope;     // 摄像头中线斜率
    double tower_distance; // 到塔的距离

    double target_distance; // 目标停车距离

    uint8_t line_found;      // 是否收到/找到线
    uint8_t junction_signal; // 是否收到 T 路口/右转信号
} Chassis_Task1;

void Chassis_Task1_Init(Chassis_Task1 *t1, Chassis *chassis);
void Chassis_Task1_SetTargetDistance(Chassis_Task1 *t1, double target_distance);
void Chassis_Task1_OnLineData(Chassis_Task1 *t1, double center, double slope);
void Chassis_Task1_OnJunctionSignal(Chassis_Task1 *t1);
void Chassis_Task1_OnTowerDistance(Chassis_Task1 *t1, double distance);
void Chassis_Task1_Update(Chassis_Task1 *t1);
Chassis_Task1_CheckPoint Chassis_Task1_GetState(const Chassis_Task1 *t1);
Chassis_Task1_HostEvent Chassis_Task1_PopHostEvent(Chassis_Task1 *t1);
uint8_t Chassis_Task1_IsDone(const Chassis_Task1 *t1);

#endif