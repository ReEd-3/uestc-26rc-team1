#include "odometry.h"
#include "chassis.h"    // 获取底盘实际速度
#include "macnum.h"     // 可能用到PI
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

Odometry_t odometry = {0};

// 外部底盘实例，用于读取实际速度
extern Chassis_t chassis;

void Odometry_Init(float start_x, float start_y, float start_yaw) {
    odometry.x = start_x;
    odometry.y = start_y;
    odometry.yaw = start_yaw;
}

void Odometry_Update(void) {
    static uint32_t last_time = 0;
    uint32_t now = HAL_GetTick();
    float dt = (now - last_time) / 1000.0f;
    if (last_time == 0) {
        last_time = now;
        return;
    }
    last_time = now;

    // 假设底盘实际速度已更新（在Chassis_UpdateActualVelocity中）
    float vx = chassis.real_vx;      // 车体系前进速度
    float vy = chassis.real_vy;      // 车体系左移速度
    float omega = chassis.real_omega; // 旋转角速度

    // 将车体系速度转换到全局坐标系（以当前yaw旋转）
    float cos_yaw = cosf(odometry.yaw);
    float sin_yaw = sinf(odometry.yaw);
    float vx_global =  cos_yaw * vx - sin_yaw * vy;
    float vy_global =  sin_yaw * vx + cos_yaw * vy;

    odometry.x += vx_global * dt;
    odometry.y += vy_global * dt;
    odometry.yaw += omega * dt;

    // 角度归一化到 [-PI, PI]
    while (odometry.yaw >  PI) odometry.yaw -= 2.0f * PI;
    while (odometry.yaw < -PI) odometry.yaw += 2.0f * PI;
}

void Odometry_SetPosition(float x, float y, float yaw) {
    odometry.x = x;
    odometry.y = y;
    odometry.yaw = yaw;
}

void Odometry_GetPosition(float *x, float *y, float *yaw) {
    if (x)  *x = odometry.x;
    if (y)  *y = odometry.y;
    if (yaw) *yaw = odometry.yaw;
}

void Odometry_FuseRadar(float radar_x, float radar_y, float radar_yaw) {
    // 简单融合：直接使用雷达数据（若雷达可靠）
    // 可加入滤波，此处示例为完全信任雷达
    odometry.x = radar_x;
    odometry.y = radar_y;
    odometry.yaw = radar_yaw;
}
