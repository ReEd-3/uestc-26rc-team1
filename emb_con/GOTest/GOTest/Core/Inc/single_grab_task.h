#ifndef __SINGLE_GRAB_TASK_H
#define __SINGLE_GRAB_TASK_H

#include <stdint.h>
#include "go8010_pos_ctrl.h"
#include "valve_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    SINGLE_GRAB_IDLE,		//空闲
    SINGLE_GRAB_MOVE_TARGET,//滑轨伸出，移动到目标高度
    SINGLE_GRAB_SUCK_WAIT,	//等待吸附延时
    SINGLE_GRAB_LIFT,		//把方块举起来
    SINGLE_GRAB_FINISH,		//整套流程走完
    SINGLE_GRAB_ERROR		//发生异常
} SingleGrabState_t;

typedef enum
{
    SINGLE_GRAB_EVENT_NONE = 0,	//没有事件
    SINGLE_GRAB_EVENT_FINISH,	//抓取成功
    SINGLE_GRAB_EVENT_ERROR		//抓取发生错误
} SingleGrabHostEvent_t;

typedef struct
{
    SingleGrabState_t        state;
    SingleGrabHostEvent_t    host_event;

    /* 硬件句柄，上层Init注入，和box_flip_task保持一致 */
    GO8010_PosCtrl_t*        pos_ctrl_slide;   //短滑轨位置控制器
    GO8010_Motor_t*          motor_slide;      //滑轨电机句柄
    GO8010_PosCtrl_t*        pos_ctrl_z;       //Z抬升位置控制器
    GO8010_Motor_t*          motor_z;          //Z轴电机句柄
    Valve_HandleTypeDef*     valve_suction;    //吸盘电磁阀

    uint8_t                  target_phy_pos;   //目标槽位0~3，用于查表Z高度
    uint32_t                 state_enter_ms;   //进入当前状态的时间戳(ms)
    uint32_t                 suck_delay_ms;    //吸附等待延时ms
	uint32_t                 release_delay_ms; //下放到位后，吸盘释放延时
} SingleGrabTask_t;

/**
 * @brief 初始化，注入硬件句柄，对齐box_flip_task
 */
void SingleGrabTask_Init(SingleGrabTask_t* task,
                         GO8010_PosCtrl_t* pos_ctrl_slide,
                         GO8010_Motor_t* motor_slide,
                         GO8010_PosCtrl_t* pos_ctrl_z,
                         GO8010_Motor_t* motor_z,
                         Valve_HandleTypeDef* valve_suction);

/**
 * @brief 修改吸附释放延时
 */
void SingleGrabTask_SetSuckDelay(SingleGrabTask_t* task, uint32_t suck_ms);
void SingleGrabTask_SetReleaseDelay(SingleGrabTask_t* task, uint32_t release_ms);

/**
 * @brief 启动抓取，传入目标pos 0~3
 */
void SingleGrabTask_Start(SingleGrabTask_t* task, uint8_t pos);

/**
 * @brief 复位任务，关闭阀、电机停止，回到IDLE
 */
void SingleGrabTask_Reset(SingleGrabTask_t* task);

/**
 * @brief 状态机周期更新
 * @param now_ms 当前系统时间戳ms
 */
void SingleGrabTask_Update(SingleGrabTask_t* task, uint32_t now_ms);

SingleGrabState_t        SingleGrabTask_GetState(const SingleGrabTask_t* task); // 获取当前内部状态，只读，不会修改task里面任何内容
SingleGrabHostEvent_t    SingleGrabTask_PopHostEvent(SingleGrabTask_t* task);	// 弹出（读取并清除）事件，会修改task内部host_event成员
uint8_t                  SingleGrabTask_IsDone(const SingleGrabTask_t* task);	// 判断任务是否结束（成功or失败都算做完），只读

#ifdef __cplusplus
}
#endif

#endif