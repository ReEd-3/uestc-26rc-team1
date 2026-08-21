#include "arm_move_nonblock.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include <math.h>

//======== 原版的宏，复制到这里，不再依赖外部motor_app.c ========
#define ARM_TICKS_PER_MM      819.2f
#define ARM_SPEED_RAW          3000.0
#define ARM_ARRIVE_THRESHOLD   300
#define ARM_PHASE_TIMEOUT_MS   60000

//======== extern 引用你工程中已经存在的变量（来自M3508驱动）========
extern M3508_CAN_All m3508_can_1;
extern double target_rpm[8];
extern osMutexId_t motorDataMutexHandle;
extern const uint16_t M3508_ENCODER_RESOLUTION;

//======== 我们自己的非阻塞上下文，本文件私有static ========
typedef enum
{
    ARM_MOVE_IDLE,
    ARM_MOVE_RUNNING,
    ARM_MOVE_TIMEOUT
} ArmMoveState_t;

typedef struct
{
    ArmMoveState_t state;
    int dir;
    double dist_ticks;
    int64_t tgt;
    int64_t travel;
    uint16_t last_pos;
    uint32_t t0;
    uint32_t t_prev;
} ArmMoveCtx_t;

static ArmMoveCtx_t g_arm_ctx;

//======== 下面四个函数实现全部粘贴进来 ========
void Arm_MoveAbort(void)
{
    osMutexAcquire(motorDataMutexHandle, osWaitForever);

    g_arm_ctx.state = ARM_MOVE_IDLE;
    g_arm_ctx.dir = 0;
    g_arm_ctx.dist_ticks = 0;
    g_arm_ctx.tgt = 0;
    g_arm_ctx.travel = 0;
    g_arm_ctx.last_pos = 0;
    g_arm_ctx.t0 = 0;
    g_arm_ctx.t_prev = 0;

    osMutexRelease(motorDataMutexHandle);
}

void Arm_LockPosition(void)
{
    osMutexAcquire(motorDataMutexHandle, osWaitForever);

    M3508_PositionPID_MotorInit(&m3508_can_1.motors[0], 12.0, 0.0, 0.15, 0.001);
    M3508_IIRFilter_SetAlpha(&m3508_can_1.motors[0], M3508_POSITIONPID_MODE, 0.85);
    M3508_PIDMode_Switch(&m3508_can_1.motors[0], M3508_POSITIONPID_MODE);

    target_rpm[0] = (double)m3508_can_1.motors[0].position;
    M3508_SetPositionTarget(&m3508_can_1, target_rpm);

    osMutexRelease(motorDataMutexHandle);
}

void Arm_MoveStart(double distance_mm)
{
    if(g_arm_ctx.state != ARM_MOVE_IDLE)
    {
        return;
    }
    if(fabs(distance_mm) < 0.01)
    {
        return;
    }

    const int dir  = (distance_mm >= 0.0) ? 1 : -1;
    const double dist = fabs(distance_mm) * ARM_TICKS_PER_MM;

    osMutexAcquire(motorDataMutexHandle, osWaitForever);

    g_arm_ctx.state = ARM_MOVE_RUNNING;
    g_arm_ctx.dir = dir;
    g_arm_ctx.dist_ticks = dist;

    g_arm_ctx.tgt = (int64_t)m3508_can_1.motors[0].position;
    g_arm_ctx.travel = 0;
    g_arm_ctx.last_pos = m3508_can_1.motors[0].position;

    g_arm_ctx.t0 = HAL_GetTick();
    g_arm_ctx.t_prev = HAL_GetTick();

    M3508_SpeedPID_MotorInit(&m3508_can_1.motors[0],    1.5, 0.05, 0.0,  0.001);
    M3508_PositionPID_MotorInit(&m3508_can_1.motors[0], 3.5, 0.0,  0.05, 0.001);
    M3508_IIRFilter_SetAlpha(&m3508_can_1.motors[0], M3508_SPEEDPID_MODE,   0.6);
    M3508_IIRFilter_SetAlpha(&m3508_can_1.motors[0], M3508_POSITIONPID_MODE, 0.8);
    M3508_PIDMode_Switch(&m3508_can_1.motors[0], M3508_CASCADE_MODE);

    target_rpm[0] = (double)g_arm_ctx.tgt;
    M3508_SetPositionTarget(&m3508_can_1, target_rpm);

    osMutexRelease(motorDataMutexHandle);
}

ArmMoveState_t Arm_Move_Update(uint32_t now_ms)
{
    ArmMoveCtx_t* ctx = &g_arm_ctx;
    if(ctx->state != ARM_MOVE_RUNNING)
    {
        return ctx->state;
    }

    if ((now_ms - ctx->t0) > ARM_PHASE_TIMEOUT_MS)
    {
        ctx->state = ARM_MOVE_TIMEOUT;
        return ctx->state;
    }

    uint32_t dt = now_ms - ctx->t_prev;
    ctx->t_prev = now_ms;
    if(dt == 0) dt = 1;

    osMutexAcquire(motorDataMutexHandle, osWaitForever);

    const double step_per_ms = ((double)ARM_SPEED_RAW * M3508_ENCODER_RESOLUTION) / 60000.0;
    ctx->tgt += (int64_t)( ctx->dir * step_per_ms * (double)dt );
    target_rpm[0] = (double)ctx->tgt;
    M3508_SetPositionTarget(&m3508_can_1, target_rpm);

    uint16_t cur = m3508_can_1.motors[0].position;
    int16_t  dl  = (int16_t)(cur - ctx->last_pos);
    ctx->last_pos = cur;
    if (dl >  4096) dl -= 8192;
    if (dl < -4096) dl += 8192;
    ctx->travel += dl;

    osMutexRelease(motorDataMutexHandle);

    if( (int64_t)(ctx->dir * ctx->travel) >= (int64_t)(ctx->dist_ticks - ARM_ARRIVE_THRESHOLD) )
    {
        ctx->state = ARM_MOVE_IDLE;
    }

    return ctx->state;
}
