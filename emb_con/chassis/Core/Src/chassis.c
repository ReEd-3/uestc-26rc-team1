#include "chassis.h"
#include "math.h"
#include <stdlib.h>

Chassis_t chassis;

#ifndef PI
#define PI 3.14159265358979323846f
#endif

static float Chassis_NormalizeAngle(float angle) {
    while (angle >  PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

void Chassis_Init(void) {
    Macnum_Init(&chassis.macnum, WHEEL_RADIUS, WHEEL_BASE_X, WHEEL_BASE_Y);

    chassis.vx = chassis.vy = chassis.omega = 0;
    chassis.real_vx = chassis.real_vy = chassis.real_omega = 0;

    PID_Init(&chassis.pid_x, 2.0f, 0.0f, 0.0f, 0.001f);
    PID_Init(&chassis.pid_y, 2.0f, 0.0f, 0.0f, 0.001f);
    PID_Init(&chassis.pid_rot, 3.0f, 0.0f, 0.0f, 0.001f);

    chassis.point_reached = false;
    chassis.arrive_pos_err = 0.05f;
    chassis.arrive_angle_err = 0.05f;
    chassis.point_tick = 0;
    chassis.point_tick_threshold = 5;

    // 循线 PID 初始化
    chassis.line_kp = 2.0f;
    chassis.line_ki = 0.0f;          // 默认为0，即PD
    chassis.line_kd = 0.5f;
    chassis.line_integral = 0.0f;
    chassis.line_integral_max = 0.5f; // 积分限幅，防止过大
    chassis.last_line_error = 0.0f;
    chassis.line_forward_speed = 0.3f;
}

void Chassis_SetBodyVelocity(float vx, float vy, float omega) {
    chassis.vx = vx;
    chassis.vy = vy;
    chassis.omega = omega;

    Macnum_SetTarget(&chassis.macnum, (double)vx, (double)vy, (double)omega);
    double target_rpm[4];
    for (int i = 0; i < 4; i++) {
        target_rpm[i] = chassis.macnum.tar_rpm[i];
    }
    M3508_SetSpeedTarget(&m3508_can_1, target_rpm);
}

void Chassis_UpdateActualVelocity(double *actual_rpm) {
    Macnum_StateUpdate(&chassis.macnum, actual_rpm);
    chassis.real_vx = (float)chassis.macnum.rea_vx;
    chassis.real_vy = (float)chassis.macnum.rea_vy;
    chassis.real_omega = (float)chassis.macnum.rea_omega;
}

bool Chassis_MoveToPoint(float target_x, float target_y, float target_yaw,
                         float current_x, float current_y, float current_yaw) {
    float world_err_x = target_x - current_x;
    float world_err_y = target_y - current_y;

    float cos_yaw = cosf(current_yaw);
    float sin_yaw = sinf(current_yaw);
    float body_err_x =  cos_yaw * world_err_x + sin_yaw * world_err_y;
    float body_err_y = -sin_yaw * world_err_x + cos_yaw * world_err_y;

    float angle_err = Chassis_NormalizeAngle(target_yaw - current_yaw);

    chassis.pid_x.target = 0;
    chassis.pid_x.current = body_err_x;
    float vx = PID_Compute(&chassis.pid_x);

    chassis.pid_y.target = 0;
    chassis.pid_y.current = body_err_y;
    float vy = PID_Compute(&chassis.pid_y);

    chassis.pid_rot.target = 0;
    chassis.pid_rot.current = angle_err;
    float omega = PID_Compute(&chassis.pid_rot);

    float dist = sqrtf(body_err_x * body_err_x + body_err_y * body_err_y);
    float max_speed = 0.5f;
    if (dist < 0.3f) {
        max_speed = 0.5f * (dist / 0.3f);
        if (max_speed < 0.05f) max_speed = 0.05f;
    }

    if (vx > max_speed) vx = max_speed; else if (vx < -max_speed) vx = -max_speed;
    if (vy > max_speed) vy = max_speed; else if (vy < -max_speed) vy = -max_speed;
    float max_omega = 2.0f;
    if (omega > max_omega) omega = max_omega; else if (omega < -max_omega) omega = -max_omega;

    Chassis_SetBodyVelocity(vx, vy, omega);

    bool pos_ok = (fabsf(body_err_x) < chassis.arrive_pos_err) &&
                  (fabsf(body_err_y) < chassis.arrive_pos_err);
    bool ang_ok = (fabsf(angle_err) < chassis.arrive_angle_err);

    if (pos_ok && ang_ok) {
        chassis.point_tick++;
        if (chassis.point_tick >= chassis.point_tick_threshold) {
            chassis.point_reached = true;
            chassis.point_tick = 0;
            chassis.pid_x.integral = 0;
            chassis.pid_y.integral = 0;
            chassis.pid_rot.integral = 0;
            return true;
        }
    } else {
        chassis.point_tick = 0;
    }
    chassis.point_reached = false;
    return false;
}

void Chassis_LineFollow(float line_error) {
    // 计算角速度：PID（若line_ki=0则为PD）
    chassis.line_integral += line_error * 0.001f;   // 假设dt=0.001
    if (chassis.line_integral > chassis.line_integral_max) chassis.line_integral = chassis.line_integral_max;
    if (chassis.line_integral < -chassis.line_integral_max) chassis.line_integral = -chassis.line_integral_max;

    float omega = chassis.line_kp * line_error +
                  chassis.line_ki * chassis.line_integral +
                  chassis.line_kd * (line_error - chassis.last_line_error);

    chassis.last_line_error = line_error;

    Chassis_SetBodyVelocity(chassis.line_forward_speed, 0.0f, omega);
}

void Chassis_Stop(void) {
    Chassis_SetBodyVelocity(0, 0, 0);
}
