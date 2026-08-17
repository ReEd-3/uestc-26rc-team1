#ifndef GRAY_SENSOR_H
#define GRAY_SENSOR_H

#include "stdint.h"
#include "stdbool.h"

#define GRAY_NUM        8       // 传感器数量
#define THRESHOLD       500     // 灰度阈值：大于此值认为白色（需实际标定）

void Gray_Init(void);
void Gray_Read(void);               // 从ADC缓冲区读取并滤波
float Gray_GetLineError(void);      // 计算线位置偏差，返回归一化值 -1.0 ~ +1.0

#endif
