#ifndef VISION_H
#define VISION_H

#include "stdint.h"
#include "stdbool.h"

typedef struct {
    bool updated;           // 是否收到新的有效数据
    char color;             // 目标颜色：'R'=红，'B'=蓝（本队颜色由宏定义）
    int x, y;               // 目标在图像中的像素坐标
    float angle;            // 目标相对机器人的角度（度）
    float distance;         // 目标距离（米），可选
    bool slope_finished;    // 上坡结束标志：true=已到达三区
    uint32_t timestamp;     // 最后更新时间（ms）
} VisionInfo_t;

extern VisionInfo_t vision;

void Vision_Init(void);
void Vision_Process(void);   // 非阻塞解析接收到的数据

#endif
