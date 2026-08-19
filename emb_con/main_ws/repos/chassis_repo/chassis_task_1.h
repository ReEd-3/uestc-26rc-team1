#ifndef CHASSIS_TASK_1_H
#define CHASSIS_TASK_1_H

#include <stdint.h>
#include "chassis.h"

/* 任务1状态机检查点 */
typedef enum {
    START,  // 初始状态
    MOVE_1,  // 第一段移动 
    MOVE_2,  // 第二段移动
    // FIND_LINE_1,  // 先找到要巡的线
    // FOLLOW_LINE_1,  // 第一段巡线，上位机发送右转信号的时候进入右转检查点
    // TURN_RIGHT_1,  // 第一次右转，里程计到达目标后向上位机发信号并且进入寻找巡线的检查点
    // FIND_LINE_2,  // 右转之后寻找巡线，收到上位机的信号之后再进入第二段巡线
    // FOLLOW_LINE_2,  // 第二段巡线，上位机发送信号后再进入第二次右转
    // TURN_RIGHT_2,  // 第二次右转，时机和第一次一样
    // FIND_LINE_3,  // 第二次右转之后寻找巡线
    // TOWER_APPROACH,  // 第三段：根据塔距微调，到位后停车（精确校准，暂时注释停用）
    TASK1_DONE,  // 任务完成

} Chassis_Task1_CheckPoint;

/* 需要上报给上位机的事件 */
typedef enum {
    CHASSIS_TASK1_HOST_EVENT_NONE = 0,
    CHASSIS_TASK1_HOST_EVENT_MOVE_DONE_1,  // 第一次移动完成
    CHASSIS_TASK1_HOST_EVENT_MOVE_DONE_2,  // 第二次移动完成
    CHASSIS_TASK1_HOST_EVENT_TASK_DONE,    // 任务1完成
    // CHASSIS_TASK1_HOST_EVENT_TURN_DONE_1,  // 第一次右转完成（旧巡线方案，注释保留）
    // CHASSIS_TASK1_HOST_EVENT_TURN_DONE_2,  // 第二次右转完成（旧巡线方案，注释保留）
} Chassis_Task1_HostEvent;

/* 任务1句柄 */
typedef struct {
    Chassis *chassis;              // 指向底盘句柄
    Chassis_Task1_CheckPoint state; // 当前状态

    Chassis_Task1_HostEvent host_event; // 待上报给上位机的事件
    uint8_t task_done_sent;             // 防止 TASK_DONE 重复上报

    // 第一段相对移动量（车体系：x=前进，y=横移，正负按底盘约定）
    double move1_x;
    double move1_y;
    // 第二段前进距离（车体系 x 方向）
    double move2_distance;

    // 主机实时输入
    // double line_center;    // 摄像头中线中心坐标（旧巡线方案，注释保留）
    // double line_slope;     // 摄像头中线斜率（旧巡线方案，注释保留）
    // double tower_distance; // 到塔的距离（精确校准，注释保留）
    // double target_distance; // 目标停车距离（精确校准，注释保留）

    // uint8_t line_found;      // 是否收到/找到线（旧巡线方案，注释保留）
    // uint8_t junction_signal; // 是否收到 T 路口/右转信号（旧巡线方案，注释保留）
} Chassis_Task1;

void Chassis_Task1_Init(Chassis_Task1 *t1, Chassis *chassis);
void Chassis_Task1_SetMove1(Chassis_Task1 *t1, double x, double y);
void Chassis_Task1_SetMove2Distance(Chassis_Task1 *t1, double distance);
// void Chassis_Task1_SetTargetDistance(Chassis_Task1 *t1, double target_distance); // 精确校准，注释保留
// void Chassis_Task1_OnLineData(Chassis_Task1 *t1, double center, double slope); // 旧巡线方案，注释保留
// void Chassis_Task1_OnJunctionSignal(Chassis_Task1 *t1); // 旧巡线方案，注释保留
// void Chassis_Task1_OnTowerDistance(Chassis_Task1 *t1, double distance); // 精确校准，注释保留
void Chassis_Task1_Update(Chassis_Task1 *t1);
Chassis_Task1_CheckPoint Chassis_Task1_GetState(const Chassis_Task1 *t1);
Chassis_Task1_HostEvent Chassis_Task1_PopHostEvent(Chassis_Task1 *t1);
uint8_t Chassis_Task1_IsDone(const Chassis_Task1 *t1);

#endif
