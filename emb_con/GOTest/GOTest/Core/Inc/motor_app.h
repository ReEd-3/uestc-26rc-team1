#ifndef __MOTOR_APP_H
#define __MOTOR_APP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Motor_AppInit(void);    // 电机初始化（CAN/PID/参数），在 main.c 的 USER CODE 2 里调用
void Motor_PIDUpdate(void);  // 单拍 PID 更新（读反馈 + 计算），PID 任务调用（锁由调用方持有）
void Motor_CurUpdate(void);  // 单拍位置斜坡 + 发电流，发电流任务调用（锁由调用方持有）
void VOFA_Send(void);        // VOFA+ 遥测，需要时在任务里调用
void Arm_MoveLock(double distance_mm);  // 匀速移动|distance_mm|mm(正=正转/负=反转)→到位锁死(直到下次调用)

extern volatile uint32_t motor_pid_count;  // 调试：PID 任务唤醒计数（Watch 窗口查看）
extern volatile uint32_t motor_cur_count;  // 调试：发电流任务唤醒计数（Watch 窗口查看）

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_APP_H */
