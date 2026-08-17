#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "stdint.h"

// 里程计坐标系：与场地全局坐标系一致，单位：米，弧度
typedef struct {
    float x;       // 全局X坐标
    float y;       // 全局Y坐标
    float yaw;     // 偏航角（弧度）
} Odometry_t;

extern Odometry_t odometry;

void Odometry_Init(float start_x, float start_y, float start_yaw);
void Odometry_Update(void);   // 每个控制周期调用，由底盘实际速度积分
void Odometry_SetPosition(float x, float y, float yaw);
void Odometry_GetPosition(float *x, float *y, float *yaw);

// 可选：融合雷达数据
void Odometry_FuseRadar(float radar_x, float radar_y, float radar_yaw);

#endif
