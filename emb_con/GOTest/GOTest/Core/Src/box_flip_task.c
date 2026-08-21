#include "box_flip_task.h"
#include "go8010_pos_ctrl.h"
#include "valve_driver.h"    // 改成你电磁阀驱动对应的头文件名

#define ARRIVE_DEBOUNCE_TICKS 3U   //需要连续3次5ms周期arrive=1才算真到位
// GO8010底层接口

void BoxFlipTask_Init(BoxFlipTask* task,
                      GO8010_PosCtrl_t* pos_ctrl,
                      GO8010_Motor_t* motor,
                      Valve_HandleTypeDef* valve)
{
    // 判断条件也要加上motor判空
    if (task == NULL || pos_ctrl == NULL || motor == NULL || valve == NULL)
    {
        return;
    }

    task->pos_ctrl = pos_ctrl;
    task->motor = motor;        // 绑定电机句柄
    task->valve = valve;

    task->state = BOX_FLIP_START;
    task->host_event = BOX_FLIP_EVENT_NONE;
    task->state_enter_tick = 0;

    task->suck_delay_ms = 250;
    task->stable_delay_ms = 200;

    Valve_Release(task->valve);
    GO8010_PosCtrl_Stop(task->pos_ctrl, task->motor);
	
	task->state_first_enter = 1;
    task->arrive_debounce_cnt = 0;
}

void BoxFlipTask_SetDelay(BoxFlipTask* task, uint32_t suck_ms, uint32_t stable_ms)//设置吸附与放下延时时长
{
    if (task == NULL)
        return;
    task->suck_delay_ms = suck_ms;
    task->stable_delay_ms = stable_ms;
}

void BoxFlipTask_Start(BoxFlipTask* task)//开始翻转任务状态
{
    if (task == NULL)
        return;
    if (task->state == BOX_FLIP_START || task->state == BOX_FLIP_DONE)
    {
        task->state = BOX_FLIP_MOTOR_HOME;
        task->state_enter_tick = 0;
		task->state_first_enter = 1;   //新增
        task->arrive_debounce_cnt = 0; //新增
    }
}

void BoxFlipTask_Reset(BoxFlipTask* task)//状态机安全重置
{
    if (task == NULL)
        return;

    task->state = BOX_FLIP_START;
    task->host_event = BOX_FLIP_EVENT_NONE;
    task->state_enter_tick = 0;

    Valve_Release(task->valve);
    GO8010_PosCtrl_Stop(task->pos_ctrl, task->motor);
	task->state_first_enter = 1;
    task->arrive_debounce_cnt = 0;
}

void BoxFlipTask_Update(BoxFlipTask* task, uint32_t now_ms)
{
    if (task == NULL || task->pos_ctrl == NULL || task->motor == NULL || task->valve == NULL)
    {
        return;
    }

    uint32_t delta = now_ms - task->state_enter_tick;
	
	//==================== 防抖代码=======================================
    uint8_t raw_arrive = GO8010_PosCtrl_CheckArrived(task->pos_ctrl);
    uint8_t stable_arrive = 0;

    if(raw_arrive == 1)
    {
        task->arrive_debounce_cnt ++;
        if(task->arrive_debounce_cnt >= ARRIVE_DEBOUNCE_TICKS)
        {
            stable_arrive = 1;
        }
    }
    else
    {
        task->arrive_debounce_cnt = 0;
        stable_arrive = 0;
    }
    //=====================================================================
	
    switch (task->state)
    {
    case BOX_FLIP_START:
        break;

    case BOX_FLIP_MOTOR_HOME:
       if(task->state_first_enter)
    {
        task->state_first_enter = 0;
        task->state_enter_tick = now_ms;
        GO8010_PosCtrl_SetTarget(task->pos_ctrl, 0.0f);
    }
    
    if (stable_arrive)
    {
        task->host_event = BOX_FLIP_EVENT_HOME_DONE;
        task->state = BOX_FLIP_WAIT_SUCK;
        task->state_enter_tick = now_ms;
        task->state_first_enter = 1;
        task->arrive_debounce_cnt = 0; //切新状态，防抖计数器清零！
        Valve_Hold (task->valve);
    }
        break;

    case BOX_FLIP_WAIT_SUCK:
        if (delta >= task->suck_delay_ms)
        {
            task->state = BOX_FLIP_MOTOR_ROTATE;
            task->state_enter_tick = now_ms;
            GO8010_PosCtrl_SetTarget (task->pos_ctrl, 3.1416f);
        }
        break;

    case BOX_FLIP_MOTOR_ROTATE:
        if (stable_arrive)
        {
            task->host_event = BOX_FLIP_EVENT_ROTATE_DONE;
            task->state = BOX_FLIP_WAIT_RELEASE;
            task->state_enter_tick = now_ms;
			task->state_first_enter = 1;     //
            task->arrive_debounce_cnt = 0;   //防抖判断置零
        }
        break;

    case BOX_FLIP_WAIT_RELEASE:
        if (delta >= task->stable_delay_ms)
        {
            Valve_Release(task->valve);
            task->host_event = BOX_FLIP_EVENT_TASK_FINISH;
            task->state = BOX_FLIP_DONE;
            task->state_enter_tick = now_ms;
        }
        break;

    case BOX_FLIP_DONE:
    default:
        break;
    }
}

BoxFlipState BoxFlipTask_GetState(const BoxFlipTask* task)
{
    if (task == NULL)
        return BOX_FLIP_START;
    return task->state;
}

BoxFlipHostEvent BoxFlipTask_PopHostEvent(BoxFlipTask* task)
{
    if (task == NULL)
        return BOX_FLIP_EVENT_NONE;

    BoxFlipHostEvent evt = task->host_event;
    task->host_event = BOX_FLIP_EVENT_NONE;
    return evt;
}

uint8_t BoxFlipTask_IsDone(const BoxFlipTask* task)
{
    if (task == NULL)
        return 0;
    return (task->state == BOX_FLIP_DONE) ? 1U : 0U;
}
