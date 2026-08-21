#include "single_grab_task.h"
#include "valve_driver.h"
/* 固定滑轨伸出脉冲，底盘已经开到抓取点，只做短伸出 */
#define SLIDE_FIX_TARGET    (800.0f)

//定义时间条目结构体，存一组横向、纵向时间
typedef struct
{
    uint32_t slide_ms;   //横向滑轨电机运行毫秒
    uint32_t z_down_ms;  //Z轴向运行毫秒
}GrabTimeItem_t;
/* pos查表台阶位置，pos0~pos3，根据实际机械修改 */
static const GrabTimeItem_t g_grab_time_table[4] =
{
    { .slide_ms = 120, .z_down_ms = 180 },  // pos0
    { .slide_ms = 120, .z_down_ms = 140 },  // pos1
    { .slide_ms = 120, .z_down_ms = 180 },  // pos2
    { .slide_ms = 120, .z_down_ms = 140 }   // pos3
};

void SingleGrabTask_Init(SingleGrabTask_t* task,
                         GO8010_PosCtrl_t* pos_ctrl_slide,
                         GO8010_Motor_t* motor_slide,
                         GO8010_PosCtrl_t* pos_ctrl_z,
                         GO8010_Motor_t* motor_z,
                         Valve_HandleTypeDef* valve_suction)
{
    if(task == NULL || pos_ctrl_slide == NULL || motor_slide == NULL
        || pos_ctrl_z == NULL || motor_z == NULL || valve_suction == NULL)
    {
        return;
    }
	
	//把上层传进来的硬件句柄保存到task结构体里面绑定，后续Update直接使用
    task->pos_ctrl_slide = pos_ctrl_slide;
    task->motor_slide = motor_slide;
    task->pos_ctrl_z = pos_ctrl_z;
    task->motor_z = motor_z;
    task->valve_suction = valve_suction;

    task->state = SINGLE_GRAB_IDLE;               //初始处于空闲
    task->host_event = SINGLE_GRAB_EVENT_NONE;    //初始无上报事件
    task->target_phy_pos = 0U;                    //目标槽位清零
    task->state_enter_ms = 0U;                    //状态进入时间戳清零
    task->suck_delay_ms = 100U;                   //默认吸附延时100ms，上层可以SetSuckDelay修改
	task->release_delay_ms = 100U;  			  // 默认释放延时100ms

	//硬件安全复位：关闭吸盘电磁阀，两个电机停止
    Valve_Release(task->valve_suction);
    /* 【预留】滑轨电机停止，后续替换滑轨驱动 */
    // GO8010_PosCtrl_Stop(task->pos_ctrl_slide, task->motor_slide);
    /* 【预留】Z轴电机停止，后续替换Z轴驱动 */
    // GO8010_PosCtrl_Stop(task->pos_ctrl_z, task->motor_z);
}

void SingleGrabTask_SetSuckDelay(SingleGrabTask_t* task, uint32_t suck_ms)//设置吸附延时时长
{
    if(task == NULL)
        return;
    task->suck_delay_ms = suck_ms;
}

void SingleGrabTask_SetReleaseDelay(SingleGrabTask_t* task, uint32_t release_ms)//设置释放延时时长
{
    if(task == NULL)
        return;
    task->release_delay_ms = release_ms;
}

void SingleGrabTask_Start(SingleGrabTask_t* task, uint8_t pos)
{
    if(task == NULL)
        return;
    if(task->state != SINGLE_GRAB_IDLE)
        return;
    if(pos > 3U)
        return;

    task->target_phy_pos = pos;
    task->state = SINGLE_GRAB_MOVE_TARGET;//滑轨伸出，移动到目标高度
    task->state_enter_ms = 0U;
    task->host_event = SINGLE_GRAB_EVENT_NONE;
}

void SingleGrabTask_Reset(SingleGrabTask_t* task)
{
    if(task == NULL)
        return;

    task->state = SINGLE_GRAB_IDLE;//切回空闲状态
    task->host_event = SINGLE_GRAB_EVENT_NONE;
    task->target_phy_pos = 0U;//清空目标
    task->state_enter_ms = 0U;//清空时间戳

    Valve_Release(task->valve_suction);
	
    /* 【预留】滑轨电机停止，后续替换为滑轨对应的驱动函数 */
    // GO8010_PosCtrl_Stop(task->pos_ctrl_slide, task->motor_slide);
    /* 【预留】Z轴电机停止，后续替换为Z轴对应的驱动函数 */
    // GO8010_PosCtrl_Stop(task->pos_ctrl_z, task->motor_z);
}

void SingleGrabTask_Update(SingleGrabTask_t* task, uint32_t now_ms)
{
    if(task == NULL || task->pos_ctrl_slide == NULL || task->motor_slide == NULL
        || task->pos_ctrl_z == NULL || task->motor_z == NULL || task->valve_suction == NULL)
    {
        return;
    }

    uint32_t delta_ms = now_ms - task->state_enter_ms;

    switch(task->state)
    {
    case SINGLE_GRAB_IDLE:
        break;

    case SINGLE_GRAB_MOVE_TARGET://要根据电机函数进行修改
        if(delta_ms == 0U)
        {
            /* pos查表得到数值；滑轨固定伸出 */
            float z_target = g_grab_time_table[task-> target_phy_pos].slide_ms;
			/* 【预留】滑轨电机运动到指定高度与伸出指定长度 */
            GO8010_PosCtrl_SetTarget(task->pos_ctrl_slide, SLIDE_FIX_TARGET);
            GO8010_PosCtrl_SetTarget(task->pos_ctrl_z, z_target);
        }
        /* 两个电机都到位 */
        if(GO8010_PosCtrl_CheckArrived(task->pos_ctrl_slide)
            && GO8010_PosCtrl_CheckArrived(task->pos_ctrl_z))//转换为3508电机到位检测函数
        {
            task->state = SINGLE_GRAB_SUCK_WAIT;
            task->state_enter_ms = now_ms;
            Valve_Hold(task->valve_suction);
        }
        break;

    case SINGLE_GRAB_SUCK_WAIT:
        if(delta_ms >= task->suck_delay_ms)
        {
            task->state = SINGLE_GRAB_LIFT;
            task->state_enter_ms = now_ms;
            /* 抬升，这里填抬升目标坐标 */
            GO8010_PosCtrl_SetTarget(task->pos_ctrl_z, 180.0f);//移动物块，后面换成3508的函数
        }
        break;

    case SINGLE_GRAB_LIFT:
        if(GO8010_PosCtrl_CheckArrived(task->pos_ctrl_z))//如果达到了制定角度，后面换成检验有没有到位的函数
        {
            task->state = SINGLE_GRAB_FINISH;
            task->state_enter_ms = now_ms;
        }
        break;

    case SINGLE_GRAB_FINISH:
        if(delta_ms == 0U)
        {
            task->host_event = SINGLE_GRAB_EVENT_FINISH;
        }
        break;

    case SINGLE_GRAB_ERROR:
        if(delta_ms == 0U)
        {
            Valve_Release(task->valve_suction);
			/* 【预留】滑轨电机停止，后续替换为滑轨对应的驱动函数 */
//            GO8010_PosCtrl_Stop(task->pos_ctrl_slide, task->motor_slide);
//            GO8010_PosCtrl_Stop(task->pos_ctrl_z, task->motor_z);
            task->host_event = SINGLE_GRAB_EVENT_ERROR;
        }
        break;

    default:
        SingleGrabTask_Reset(task);
        break;
    }
}

SingleGrabState_t SingleGrabTask_GetState(const SingleGrabTask_t* task)//获取状态
{
    if(task == NULL)
        return SINGLE_GRAB_IDLE;
    return task->state;
}

SingleGrabHostEvent_t SingleGrabTask_PopHostEvent(SingleGrabTask_t* task)//弹出当前事件
{
    if(task == NULL)
        return SINGLE_GRAB_EVENT_NONE;
    SingleGrabHostEvent_t evt = task->host_event;
    task->host_event = SINGLE_GRAB_EVENT_NONE;
    return evt;
}

uint8_t SingleGrabTask_IsDone(const SingleGrabTask_t* task)
{
    if(task == NULL)
        return 0U;
    return (task->state == SINGLE_GRAB_FINISH || task->state == SINGLE_GRAB_ERROR) ? 1U : 0U;
}