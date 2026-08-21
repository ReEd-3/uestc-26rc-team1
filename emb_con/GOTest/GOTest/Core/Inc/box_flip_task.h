#ifndef BOX_FLIP_TASK_H
#define BOX_FLIP_TASK_H

#include <stdint.h>
#include "go8010_pos_ctrl.h"
#include "valve_driver.h"

/* 任务1状态机检查点 */
typedef enum
{
    BOX_FLIP_START,       //初始状态
    BOX_FLIP_MOTOR_HOME,  //电机归0
    BOX_FLIP_WAIT_SUCK,   //电磁阀通电
    BOX_FLIP_MOTOR_ROTATE,//旋转180
    BOX_FLIP_WAIT_RELEASE,//电磁阀断电
    BOX_FLIP_DONE
} BoxFlipState;

typedef enum
{
    BOX_FLIP_EVENT_NONE = 0,    //无事件
    BOX_FLIP_EVENT_HOME_DONE,   //电机归0完成
    BOX_FLIP_EVENT_ROTATE_DONE,	//翻转完成
    BOX_FLIP_EVENT_TASK_FINISH  //任务完成
} BoxFlipHostEvent;



typedef struct
{
    BoxFlipState         state;				// 当前走到哪个步骤（回零/吸附/旋转/释放...）
    BoxFlipHostEvent     host_event;		// 向上层反馈事件：回零完成、旋转完成、任务结束

    GO8010_PosCtrl_t*    pos_ctrl;  		// 电机结构体
	GO8010_Motor_t*      motor;        		// 底层电机句柄（能否正常调用？）
    Valve_HandleTypeDef* valve;  			// 电磁阀结构体

    uint32_t             state_enter_tick;  // 时间戳，用来计时
    uint32_t             suck_delay_ms;     //吸住物体需要等待多少时间
    uint32_t             stable_delay_ms;   //翻转到位之后延时多久再松开
	uint8_t state_first_enter;
    uint8_t arrive_debounce_cnt;
} BoxFlipTask;

// 初始化形参同步修改
void BoxFlipTask_Init(BoxFlipTask* task,
                      GO8010_PosCtrl_t* pos_ctrl,
                      GO8010_Motor_t* motor,
                      Valve_HandleTypeDef* valve);								   //初始化函数
void BoxFlipTask_SetDelay(BoxFlipTask* task, uint32_t suck_ms, uint32_t stable_ms);//设置吸附与翻转后的延时时长的函数
void BoxFlipTask_Start(BoxFlipTask* task);										   //启动翻转动作
void BoxFlipTask_Reset(BoxFlipTask* task);										   //紧急复位
void BoxFlipTask_Update(BoxFlipTask* task, uint32_t now_ms);					   //heart


BoxFlipState        BoxFlipTask_GetState(const BoxFlipTask* task);					//获取任务状态
BoxFlipHostEvent    BoxFlipTask_PopHostEvent(BoxFlipTask* task); 					//重要事件结束
uint8_t             BoxFlipTask_IsDone(const BoxFlipTask* task);					//快捷判断，任务是否做完

#endif
