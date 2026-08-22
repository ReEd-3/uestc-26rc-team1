#include "chassis.h"
#include "app.h"
#include <math.h>

// 有关保护变量读写的头文件
#include "FreeRTOS.h"
#include "task.h"

#ifndef CHASSIS_PI
#define CHASSIS_PI 3.14159265358979323846
#endif

/* 将角度（弧度）归一化到 [-pi, pi] 范围 */
static double Chassis_AngleNormalize(double angle)
{
    while (angle > CHASSIS_PI) {
        angle -= 2.0 * CHASSIS_PI;
    }
    while (angle < -CHASSIS_PI) {
        angle += 2.0 * CHASSIS_PI;
    }
    return angle;
}

/* 重置 x/y/yaw 三个位置环 PID 的积分和误差状态 */
static void Chassis_ResetPid(Chassis *ch)
{
    if (ch == NULL) {
        return;
    }

    PID_Init(&ch->pid_x, ch->pid_x.Kp, ch->pid_x.Ki, ch->pid_x.Kd, ch->pid_x.dt);
    PID_Init(&ch->pid_y, ch->pid_y.Kp, ch->pid_y.Ki, ch->pid_y.Kd, ch->pid_y.dt);
    PID_Init(&ch->pid_yaw, ch->pid_yaw.Kp, ch->pid_yaw.Ki, ch->pid_yaw.Kd, ch->pid_yaw.dt);
}

/* 初始化底盘句柄并保存电机、麦轮和 PID 参数 */
void Chassis_Init(Chassis *ch, M3508_CAN_All *m3508, EncoderOdo *eo, const App_Config *cfg)
{
    if (ch == NULL || m3508 == NULL || cfg == NULL) {
        return;
    }

    // 电机总线
    ch->m3508 = m3508;
    ch->eo = eo;

    // 麦轮初始化
    Macnum_Init(&ch->mn,
                cfg->chassis_wheel_radius,
                cfg->half_wheelbase,
                cfg->half_track,
                cfg->gear_ratio);
    
    // 初始底盘停止
    ch->mode = CHASSIS_MODE_STOP;

    // 控制周期，速度限幅，误差范围
    ch->dt = cfg->chassis_dt;
    ch->max_vx = cfg->max_vx;
    ch->max_vy = cfg->max_vy;
    ch->max_omega = cfg->max_omega;
    ch->tol_xy = cfg->tol_xy;
    ch->tol_yaw = cfg->tol_yaw;

    PID_Init(&ch->pid_x, cfg->pos_kp, cfg->pos_ki, cfg->pos_kd, cfg->chassis_dt);
    PID_Init(&ch->pid_y, cfg->pos_kp, cfg->pos_ki, cfg->pos_kd, cfg->chassis_dt);
    PID_Init(&ch->pid_yaw, cfg->yaw_kp, cfg->yaw_ki, cfg->yaw_kd, cfg->chassis_dt);
}

/* 直接设置当前全局位姿，用于外部给定起点坐标 */
void Chassis_SetPose(Chassis *ch, double x, double y, double yaw)
{
    if (ch == NULL) {
        return;
    }

    ch->mn.rea_x = x;
    ch->mn.rea_y = y;
    ch->mn.yaw = yaw;
    if (ch->eo != NULL) {
        ch->eo->abs_x = x;
        ch->eo->abs_y = y;
    }
    Chassis_ResetPid(ch);
}

/* 用当前编码器值重置里程计零点，防止启动瞬间跳变 */
void Chassis_ResetPose(Chassis *ch, const uint16_t encoder_now[4])
{
    if (ch == NULL || encoder_now == NULL) {
        return;
    }

    Macnum_PositionReset(&ch->mn, (uint16_t *)encoder_now);
    if (ch->eo != NULL) {
        EncoderOdo_SetBeginCnt(ch->eo);
    }
    Chassis_ResetPid(ch);
}

/* 任务每个环节开始时重置码盘坐标系：下一次收到的码盘值作为零点 */
void Chassis_ResetEncoderPose(Chassis *ch)
{
    if (ch == NULL || ch->eo == NULL) {
        return;
    }

    EncoderOdo_SetBeginCnt(ch->eo);
    Chassis_ResetPid(ch);
}

/* 切换到速度模式，并设置车体目标速度 */
void Chassis_SetVelocity(Chassis *ch, double vx, double vy, double omega)
{
    if (ch == NULL) {
        return;
    }

    ch->mode = CHASSIS_MODE_VELOCITY;
    ch->tar_vx = vx;
    ch->tar_vy = vy;
    ch->tar_omega = omega;
}

/* 按车体坐标系设置相对位移目标，内部转换成绝对目标 */
void Chassis_MoveRelative(Chassis *ch, double dx, double dy, double dyaw)
{
    if (ch == NULL) {
        return;
    }

    /*
     * 车体坐标系相对移动：
     * dx 表示车体前进方向，dy 表示车体横移方向。
     * 先转换到世界坐标，再作为绝对目标控制。
     */
    double cos_yaw = cos(ch->mn.yaw);
    double sin_yaw = sin(ch->mn.yaw);

    ch->target_x = ch->eo->abs_x + dx * cos_yaw - dy * sin_yaw;
    ch->target_y = ch->eo->abs_y + dx * sin_yaw + dy * cos_yaw;
    ch->target_yaw = Chassis_AngleNormalize(ch->mn.yaw + dyaw);

    ch->mode = CHASSIS_MODE_RELATIVE_MOVE;
    Chassis_ResetPid(ch);
}

/* 设置全局坐标系下的绝对位置和姿态目标 */
void Chassis_MoveAbsolute(Chassis *ch, double x, double y, double yaw)
{
    if (ch == NULL) {
        return;
    }

    ch->target_x = x;
    ch->target_y = y;
    ch->target_yaw = Chassis_AngleNormalize(yaw);

    ch->mode = CHASSIS_MODE_ABSOLUTE_MOVE;
    Chassis_ResetPid(ch);
}

/* 轨迹跟踪用：只更新绝对位置目标，不重置PID，避免每周期积分清零 */
void Chassis_FollowTarget(Chassis *ch, double x, double y, double yaw)
{
    if (ch == NULL) {
        return;
    }

    ch->target_x = x;
    ch->target_y = y;
    ch->target_yaw = Chassis_AngleNormalize(yaw);

    ch->mode = CHASSIS_MODE_ABSOLUTE_MOVE;
    /* 注意：不调用 Chassis_ResetPid() */
}

/* 停止底盘运动并清空目标速度 */
void Chassis_Stop(Chassis *ch)
{
    if (ch == NULL) {
        return;
    }

    ch->mode = CHASSIS_MODE_STOP;
    ch->tar_vx = 0.0;
    ch->tar_vy = 0.0;
    ch->tar_omega = 0.0;
    Chassis_ResetPid(ch);
}

/* 每个控制周期调用一次：更新里程计并输出电机速度环目标 */
void Chassis_Update(Chassis *ch)
{
    if (ch == NULL || ch->m3508 == NULL) {
        return;
    }

    // 先读取当前电机反馈，用于里程计
    // M3508_ReadStatus(ch->m3508);
    uint16_t encoder_raw[4] = {
        ch->m3508->motors[0].position,
        ch->m3508->motors[1].position,
        ch->m3508->motors[2].position,
        ch->m3508->motors[3].position
    };

    /* 更新里程计，rea_x/rea_y/yaw 会更新 */
    Macnum_PositionStateUpdate(&ch->mn, encoder_raw);
    // EncoderOdo_Update(ch->eo);

    double cur_x, cur_y;

    // 只在这里临界区读一次，防止写入同时读取
    taskENTER_CRITICAL();
    cur_x = ch->eo->abs_x;
    cur_y = ch->eo->abs_y;
    taskEXIT_CRITICAL();

    double vx_cmd = 0.0;
    double vy_cmd = 0.0;
    double omega_cmd = 0.0;

    // 计算PID
    switch (ch->mode) {
        case CHASSIS_MODE_VELOCITY:
            vx_cmd = ch->tar_vx;
            vy_cmd = ch->tar_vy;
            omega_cmd = ch->tar_omega;
            break;

        case CHASSIS_MODE_RELATIVE_MOVE:
        case CHASSIS_MODE_ABSOLUTE_MOVE:
        {
            ch->pid_x.dt = ch->dt;
            ch->pid_y.dt = ch->dt;
            ch->pid_yaw.dt = ch->dt;

            double err_x = ch->target_x - cur_x;
            double err_y = ch->target_y - cur_y;
            double err_yaw = Chassis_AngleNormalize(ch->target_yaw - ch->mn.yaw);

            ch->pid_x.target = err_x;
            ch->pid_x.current = 0.0;
            ch->pid_y.target = err_y;
            ch->pid_y.current = 0.0;
            ch->pid_yaw.target = err_yaw;
            ch->pid_yaw.current = 0.0;

            double vx_world = PID_Compute(&ch->pid_x);
            double vy_world = PID_Compute(&ch->pid_y);
            double omega = PID_Compute(&ch->pid_yaw);

            /* 世界系速度命令旋转到车体系 */
            double cos_yaw = cos(ch->mn.yaw);
            double sin_yaw = sin(ch->mn.yaw);
            vx_cmd = vx_world * cos_yaw + vy_world * sin_yaw;
            vy_cmd = -vx_world * sin_yaw + vy_world * cos_yaw;
            omega_cmd = omega;
            break;
        }

        case CHASSIS_MODE_STOP:
        default:
            vx_cmd = 0.0;
            vy_cmd = 0.0;
            omega_cmd = 0.0;
            break;
    }

    /* 速度限幅 */
    if (vx_cmd > ch->max_vx)      vx_cmd = ch->max_vx;
    else if (vx_cmd < -ch->max_vx) vx_cmd = -ch->max_vx;
    if (vy_cmd > ch->max_vy)      vy_cmd = ch->max_vy;
    else if (vy_cmd < -ch->max_vy) vy_cmd = -ch->max_vy;
    if (omega_cmd > ch->max_omega)      omega_cmd = ch->max_omega;
    else if (omega_cmd < -ch->max_omega) omega_cmd = -ch->max_omega;

    /* 目标速度 -> 四个轮子 rpm -> 电机速度环 */
    Macnum_SetTarget(&ch->mn, vx_cmd, vy_cmd, omega_cmd);
    M3508_SetSpeedTarget(ch->m3508, ch->mn.tar_rpm);

    // 设置目标后再更新 PID，避免当前拍仍使用上一拍目标导致输出 0
    M3508_PID_Update(ch->m3508);

    M3508_CAN_CurrentUpdate(ch->m3508);
}

/* 判断当前是否已到达位置模式的目标点 */
uint8_t Chassis_Arrived(Chassis *ch)
{
    if (ch == NULL) {
        return 0;
    }

    if (ch->mode != CHASSIS_MODE_RELATIVE_MOVE &&
        ch->mode != CHASSIS_MODE_ABSOLUTE_MOVE) {
        return 0;
    }

    double err_x = ch->target_x - ch->eo->abs_x;
    double err_y = ch->target_y - ch->eo->abs_y;
    double err_yaw = Chassis_AngleNormalize(ch->target_yaw - ch->mn.yaw);

    return (fabs(err_x) <= ch->tol_xy &&
            fabs(err_y) <= ch->tol_xy &&
            fabs(err_yaw) <= ch->tol_yaw) ? 1 : 0;
}