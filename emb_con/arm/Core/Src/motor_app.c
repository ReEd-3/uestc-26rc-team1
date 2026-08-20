#include "motor_app.h"
#include "main.h"
#include <stdio.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

extern osMutexId_t motorDataMutexHandle;   /* freertos.c 创建；保护 m3508_can_1 共享电机数据 */

static double target_rpm[8] = {0};// 8 个电机的目标位置（计数值，0~8191）
static M3508_CAN_All m3508_can_1;// 8 个 M3508 电机的句柄数组，使用 CAN1
static char vofa_buf[64];// VOFA+ 遥测缓冲区
volatile uint32_t motor_pid_count = 0;// 调试：PID 任务唤醒计数（应约 1000/s），全局便于 Watch 查看
volatile uint32_t motor_cur_count = 0;// 调试：发电流任务唤醒计数（应约 1000/s），全局便于 Watch 查看
volatile int m3508_init_status = -1;   // 调试：M3508_CAN_Init 结果 0=成功 1=失败（-1=未执行），Watch 查看
volatile int m3508_tx_status   = -1;   // 调试：最近一次电流帧发送结果 0=已入队 1=发送失败，Watch 查看

volatile uint8_t motor_pos_ready = 0;   // PID 首拍位置吸附完成标志

/* 调试镜像（全局变量）：把 static m3508_can_1 里的关键值同步出来，
   这样在任意断点都能直接 Watch，不会出现"0 和报错循环" */
volatile uint16_t dbg_position      = 0;   // 当前位置（计数）
volatile int16_t  dbg_speed         = 0;   // 实际转速（原始RPM）
volatile double   dbg_pos_target    = 0;   // 位置环目标
volatile double   dbg_speed_target  = 0;   // 速度环目标（原始RPM）
volatile int16_t  dbg_current       = 0;   // 反馈电流
volatile int16_t  dbg_cur_low       = 0;   // 实际发送的电流指令+

/* 机械臂动作阶段指示（仅调试）：0=未开始 1=正转中 2=锁定中 3=反转中 4=回位锁死 5=完成 */
volatile uint8_t  dbg_arm_phase     = 0;
volatile int64_t  dbg_arm_travel    = 0;   // 当前阶段累计转角（调试）


/*  电机初始化（串级测试：位置外环→速度内环→电流；转动 TEST_MOVE_DIST 后锁死） */
void Motor_AppInit(void) {
    if (M3508_CAN_Init(&m3508_can_1, (1 << 0), &hfdcan1) != HAL_OK) {// 只初始化 ID=1 的电机；失败则卡死便于排查（常见原因：FDCAN StdFiltersNbr 不足 8）
        m3508_init_status = 1;
        Error_Handler();
    }
    m3508_init_status = 0;




    //M3508_SpeedPID_Init(&m3508_can_1, 2.0, 0.1, 0.0, 0.001);// 速度环 PID 参数设置
    //M3508_PositionPID_Init(&m3508_can_1, 6, 0.0, 0.1, 0.001);// 位置环 PID 参数设置

    M3508_PID_SetIntLim(&m3508_can_1.motors[0], M3508_SPEEDPID_MODE, 600);// 速度环积分限幅
    M3508_PID_SetIntLim(&m3508_can_1.motors[0], M3508_POSITIONPID_MODE, 600);// 位置环积分限幅

    //M3508_IIRFilter_SetAlpha(&m3508_can_1.motors[0], M3508_SPEEDPID_MODE, 0.8);// 速度环滤波
    //M3508_IIRFilter_SetAlpha(&m3508_can_1.motors[0], M3508_POSITIONPID_MODE, 0.9);// 位置环滤波（压静止抖动）

    target_rpm[0] = 0; // 位置目标初始 0，首拍会吸附到当前位置
    M3508_SetPositionTarget(&m3508_can_1, target_rpm);// 写入位置环目标
    //M3508_PIDMode_Switch(&m3508_can_1.motors[0], M3508_CASCADE_MODE);// 串级模式：位置外环→速度内环
    /* 速度目标限幅（原始RPM）：驱动层已做 ×19.2 换算，此限幅为原始RPM；
       350≈18 输出RPM 太慢，3000≈156 输出RPM 适合观察 */
    m3508_can_1.motors[0].max_speed = 10000;
}

/* PID 任务调用 */
void Motor_PIDUpdate(void) {
    motor_pid_count++;  // 调试：PID 任务被唤醒一次

    /* 等第一帧真实反馈：首帧未到前 position 是初始化值 0，
       此时吸附会把目标吸成 0，真实位置一进来就往 0 位反向转动+震动*/

    if (!M3508_IsFeedbackReady()) {
        M3508_ReadStatus(&m3508_can_1);
    }

    M3508_PID_Update(&m3508_can_1); // 串级：读反馈 + 双环计算

    /* 同步调试镜像（每 1ms 刷新一次，供任意断点 Watch） */
    dbg_position     = m3508_can_1.motors[0].position;
    dbg_speed        = m3508_can_1.motors[0].speed;
    dbg_pos_target   = m3508_can_1.motors[0].position_pid.target;
    dbg_speed_target = m3508_can_1.motors[0].speed_pid.target;
    dbg_current      = m3508_can_1.motors[0].current;
    dbg_cur_low      = m3508_can_1.cur_low[0];
}

/* 发送缓存电流任务调用*/
void Motor_CurUpdate(void) {
    motor_cur_count++;
    m3508_tx_status = (M3508_SetCurrent(&m3508_can_1) != HAL_OK) ? 1 : 0;
}
/*  VOFA+ 遥测  */
void VOFA_Send(void) {
    int len = sprintf(vofa_buf, "%d,%d,%d,%d\n",
        (int)m3508_can_1.motors[0].position_pid.target,  // 目标位置
        (int)m3508_can_1.motors[0].position,             // 当前位置
        (int)m3508_can_1.motors[0].speed_pid.target,     // 目标速度（RPM）
        (int)m3508_can_1.motors[0].current               // 电流
      );
    HAL_UART_Transmit_IT(&huart3, (uint8_t *)vofa_buf, len);
}


/* ================================================================
 * 3508 单段动作：匀速移动 |distance_mm| 毫米 → 到位锁死（直到下一次调用）
 * 参数：distance_mm = 移动距离（mm，带方向）：正值=正转，负值=反转；0 不动作
 * 换算：mm → 转子计数用 ARM_TICKS_PER_MM（沿用之前验证过的 819.2 计数/mm；
 *       结合 1 输出圈=157286 计数，即约 192mm/输出圈，按你实际机械修改此值）
 * 行为：只做"朝 distance_mm 方向移动 |distance_mm| 毫米 + 锁死"这一段，不做自动回程；
 *       返回后电机保持在终点锁定，直到下一次调用（方向由下次的正负决定）。
 * 说明：阻塞式（在调用它的任务里等待动作完成）；
 *       PID 任务与发电流任务必须正常运行（Motor_PIDUpdate/Motor_CurUpdate）。
 * ================================================================ */
#define ARM_TICKS_PER_MM      819.2f   /* mm→转子计数换算（1 输出圈=157286 计数≈192mm，按机械改） */
#define ARM_SPEED_RAW          3000.0  /* 匀速目标速度 */
#define ARM_ARRIVE_THRESHOLD   300     /* 到位判定阈值（计数） */
#define ARM_PHASE_TIMEOUT_MS   60000   /* 每段超时保护（防卡死） */
#define ARM_FEEDBACK_TIMEOUT_MS 2000     /* 等待反馈就绪上限（超时=硬件问题，软件不动作） */

volatile uint32_t dbg_arm_alive = 0;     /* 心跳：移动段每轮自增，Watch 看它在涨=在跑 */

void Arm_MoveLock(double distance_mm)
{

	dbg_arm_alive++;

	/* 1) 有界等待反馈就绪 */
	    uint32_t tw = HAL_GetTick();
	    while (!M3508_IsFeedbackReady()) {
	        if ((HAL_GetTick() - tw) > ARM_FEEDBACK_TIMEOUT_MS) { dbg_arm_phase = 0; return; }
	        vTaskDelay(pdMS_TO_TICKS(5));
	    }
	    if (distance_mm == 0.0) return;

    const int    dir  = (distance_mm >= 0.0) ? 1 : -1;   /* 正=正转；负=反转 */
    const double dist = (distance_mm >= 0.0 ? distance_mm : -distance_mm) * ARM_TICKS_PER_MM; /* 移动距离（转子计数>0） */

    /* 2) 移动段：串级位置斜坡 —— 时间比例推进（抗调度抖动），1ms 粒度 */

        /* ===== 移动段参数：平滑跟坡（慢斜坡 + 低微分） ===== */
            M3508_SpeedPID_MotorInit(&m3508_can_1.motors[0],    1.5, 0.05, 0.0,  0.001);
            M3508_PositionPID_MotorInit(&m3508_can_1.motors[0], 3.5, 0.0,  0.05, 0.001);
            M3508_IIRFilter_SetAlpha(&m3508_can_1.motors[0], M3508_SPEEDPID_MODE,   0.6);
            M3508_IIRFilter_SetAlpha(&m3508_can_1.motors[0], M3508_POSITIONPID_MODE, 0.8);

        M3508_PIDMode_Switch(&m3508_can_1.motors[0], M3508_CASCADE_MODE);
        target_rpm[0] = m3508_can_1.motors[0].position;
        M3508_SetPositionTarget(&m3508_can_1, target_rpm);

        const double step_per_ms = ((double)ARM_SPEED_RAW * M3508_ENCODER_RESOLUTION) / 60000.0; /* 计数/ms（800→109） */

        int64_t  travel = 0;
        uint16_t last   = m3508_can_1.motors[0].position;
        int64_t  tgt    = (int64_t)target_rpm[0];
        uint32_t t0     = HAL_GetTick();
        uint32_t t_prev = t0;

        while ((int64_t)(dir * travel) < (int64_t)(dist - ARM_ARRIVE_THRESHOLD)) {
            uint32_t now = HAL_GetTick();
            uint32_t dt  = now - t_prev;          /* 实际流逝时间（ms，无符号减法自动处理回绕） */
            t_prev = now;
            if (dt == 0) dt = 1;

            osMutexAcquire(motorDataMutexHandle, osWaitForever);
            tgt += (int64_t)(dir * step_per_ms * (double)dt);   /* 时间比例推进：速度恒定不随调度抖动 */
            target_rpm[0] = (double)tgt;
            M3508_SetPositionTarget(&m3508_can_1, target_rpm);
            {   /* 真实反馈累计已走距离（过零处理） */
                uint16_t cur = m3508_can_1.motors[0].position;
                int16_t  dl  = (int16_t)(cur - last);
                last = cur;
                if (dl >  4096) dl -= 8192;
                if (dl < -4096) dl += 8192;
                travel += dl;
            }
            osMutexRelease(motorDataMutexHandle);

            dbg_arm_travel = travel;
            dbg_arm_alive++;
            if ((HAL_GetTick() - t0) > ARM_PHASE_TIMEOUT_MS) break;
            vTaskDelay(pdMS_TO_TICKS(1));         /* 1ms 粒度 → 速度目标每 1ms 平滑爬升 */
        }

    /* 3) 停斜坡 → 等物理停稳（留惯性余量） */
    osMutexAcquire(motorDataMutexHandle, osWaitForever);
    target_rpm[0] = last;                              /* 停住目标，让过冲自然收敛 */
    M3508_SetPositionTarget(&m3508_can_1, target_rpm);
    osMutexRelease(motorDataMutexHandle);
    vTaskDelay(pdMS_TO_TICKS(150));

    /* 4) 锁死段：直接位置环（刚度最大），保持到下次调用 */
    //dbg_arm_phase = 2;

    /* ===== 锁死段参数：高刚度 + 阻尼（死区在驱动层已加，静止不抖） ===== */
    M3508_PositionPID_MotorInit(&m3508_can_1.motors[0], 12.0, 0.0, 0.15, 0.001);
    M3508_IIRFilter_SetAlpha(&m3508_can_1.motors[0], M3508_POSITIONPID_MODE, 0.85);

    M3508_PIDMode_Switch(&m3508_can_1.motors[0], M3508_POSITIONPID_MODE);
    osMutexAcquire(motorDataMutexHandle, osWaitForever);
    target_rpm[0] = m3508_can_1.motors[0].position;   /* 锁在停稳后的真实位置 */
    M3508_SetPositionTarget(&m3508_can_1, target_rpm);
    osMutexRelease(motorDataMutexHandle);

    //dbg_arm_phase = 3;                                 /* 完成；返回后保持锁定 */
    /* PID/发电流任务继续运行 → 保持锁定，直到下次调用 */
}
