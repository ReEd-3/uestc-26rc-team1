#ifndef ROBOT_TASK_H
#define ROBOT_TASK_H

#include "stdint.h"
#include "stdbool.h"

typedef enum {
    STATE_INIT,
    STATE_MOVE_TO_ZONE1,            // 从启动区点对点移动到一区（无白线）
    STATE_ZONE1_APPROACH_TOWER,     // 对准初始塔
    STATE_ZONE1_FLIP_KFS,           // 翻转顶部KFS
    STATE_MOVE_TO_LINE,             // 任务一完成后朝固定方向移动至白线区（利用编码器）
    STATE_LINE_FOLLOW_TO_ZONE2,     // 巡线前往二区
    STATE_ZONE2_FIND_KFS,
    STATE_ZONE2_GRAB_KFS,
    STATE_CARRY_TO_ZONE3,
    STATE_ZONE3_PLACE_KFS,
    STATE_DONE,
    STATE_ERROR
} RobotState_t;

void RobotTask_Init(void);
void RobotTask_Run(void);

#endif
