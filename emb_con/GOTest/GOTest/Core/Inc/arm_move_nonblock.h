#ifndef __ARM_MOVE_NONBLOCK_H
#define __ARM_MOVE_NONBLOCK_H

#include <stdint.h>

typedef enum
{
    ARM_MOVE_IDLE,      //空闲状态
    ARM_MOVE_RUNNING,	//正在移动
    ARM_MOVE_TIMEOUT 	//移动超时失败
} ArmMoveState_t;

void Arm_MoveStart(double distance_mm);			//设置运动目标位置，仅仅写变量，不做PID计算
ArmMoveState_t Arm_Move_Update(uint32_t now_ms);//计算运动时间、判断是否到位、是否超时
 
void Arm_MoveAbort(void);
void Arm_LockPosition(void);

#endif
