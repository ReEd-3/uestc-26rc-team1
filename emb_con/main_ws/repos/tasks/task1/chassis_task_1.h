#ifndef CHASSIS_TASK_1_H
#define CHASSIS_TASK_1_H

#include <stdint.h>
#include "chassis.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 任务1状态机检查点 */
typedef enum {
    START,  // 初始状态
    MOVE_1,  // 第一段移动 
    MOVE_2,  // 第二段移动
    CUBE_GET_1,  // 第一次抓取
    MOVE_3,  // 第三段移动
    TASK1_DONE,  // 任务完成

} Chassis_Task1_CheckPoint;

/* 需要上报给上位机的事件 */
typedef enum {
    CHASSIS_TASK1_HOST_EVENT_NONE = 0,
    CHASSIS_TASK1_HOST_EVENT_MOVE_DONE_1,  // 第一次移动完成
    CHASSIS_TASK1_HOST_EVENT_MOVE_DONE_2,  // 第二次移动完成
    CHASSIS_TASK1_HOST_EVENT_MOVE_DONE_3,  // 第三次移动完成
    CHASSIS_TASK1_HOST_EVENT_ARMGET_DONE,  // 机械臂抓取完成
    CHASSIS_TASK1_HOST_EVENT_TASK_DONE,    // 任务1完成
} Chassis_Task1_HostEvent;

/* 任务1句柄 */
typedef struct {
    Chassis *chassis;              // 指向底盘句柄
    Chassis_Task1_CheckPoint state; // 当前状态

    Chassis_Task1_HostEvent host_event; // 待上报给上位机的事件
    uint8_t task_done_sent;             // 防止 TASK_DONE 重复上报

    // 第一段相对移动量
    double move1_x;
    double move1_y;
    // 第二段前进距离
    double move2_x;
    double move2_y;
    // 第二段前进距离
    double move3_x;
    double move3_y;

} Chassis_Task1;

void Chassis_Task1_Init(Chassis_Task1 *t1, Chassis *chassis);
void Chassis_Task1_SetMove1(Chassis_Task1 *t1, double x, double y);
void Chassis_Task1_SetMove2(Chassis_Task1 *t1, double x, double y);
void Chassis_Task1_SetMove3(Chassis_Task1 *t1, double x, double y);
void Chassis_Task1_Update(Chassis_Task1 *t1);
Chassis_Task1_CheckPoint Chassis_Task1_GetState(const Chassis_Task1 *t1);
Chassis_Task1_HostEvent Chassis_Task1_PopHostEvent(Chassis_Task1 *t1);
uint8_t Chassis_Task1_IsDone(const Chassis_Task1 *t1);

#ifdef __cplusplus
}
#endif

#endif
