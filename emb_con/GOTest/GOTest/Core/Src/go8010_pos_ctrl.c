#include "go8010_pos_ctrl.h"
#include <math.h>
//现有小问题，没有延时，状态切换之后要预留时间让电机转到正确角度
/**
 * @brief 控制器初始化，加载你原先调试好的默认参数
 */
void GO8010_PosCtrl_Init(GO8010_PosCtrl_t *hctrl)
{
    hctrl->k_p_far         = 1.0f;
    hctrl->k_p_near        = 5.0f;
    hctrl->k_w             = 0.10f;
    hctrl->pos_target_out  = 0.0f;   // 默认置0
    hctrl->brake_threshold = 0.30f;
    hctrl->hyst_band       = 0.05f;

    hctrl->use_rot_spd     = 0.0f;
    hctrl->feed_torque     = 0.0f;
    hctrl->rotor_pos_cmd   = 0.0f;
    hctrl->flag_approaching= 1U;
	hctrl->real_pos_out    = 0.0f;
	hctrl->real_pos        = 0.0f;
	// =========先给一组保守初始值，后续现场微调=========
    hctrl->pos_arrive_thr = 0.08f;    // ≈4.6°
    hctrl->spd_arrive_thr = 0.15f;    // 低于该转速认为静止
    hctrl->arrive_filter_frame = 5U;  // 连续5帧满足条件才算到位
}

/**
 * @brief 设置目标角度：输出轴 180°(π rad)
 */
void GO8010_PosCtrl_SetTarget180(GO8010_PosCtrl_t *hctrl)
{
    hctrl->pos_target_out = 3.1416f;
}

/**
 * @brief 设置目标角度：输出轴 90°(π/2 rad)
 */
void GO8010_PosCtrl_SetTarget90(GO8010_PosCtrl_t *hctrl)
{
    hctrl->pos_target_out = 1.5708f;
}

/**
 * @brief 单次位置控制迭代，完整迁移你while循环内控制逻辑
 */
int GO8010_PosCtrl_Run(GO8010_PosCtrl_t *hctrl, GO8010_Motor_t *hmotor)
{
    
    float real_v      = 0.0f;

    // 下发上一轮缓存指令
    GO8010_Motor_SetHybrid(hmotor, hctrl->feed_torque, hctrl->use_rot_spd,
                           hctrl->rotor_pos_cmd,
                           hctrl->flag_approaching ? hctrl->k_p_far : hctrl->k_p_near,
                           hctrl->k_w);

    int comm_state = GO8010_Motor_SendRecv(hmotor, 20);

    if(comm_state == 1)
    {
		hctrl->real_pos = 0.0f;
        hctrl->real_pos  = GO8010_Motor_GetPos(hmotor);
        real_v   = GO8010_Motor_GetVel(hmotor);
		hctrl->real_v_out = real_v / MOTOR_GEAR_RATIO;

        hctrl-> real_pos_out = hctrl->real_pos  / MOTOR_GEAR_RATIO;//负载端真实角度
        float pos_err      = hctrl->pos_target_out - hctrl->real_pos_out;
        float abs_err      = fabs(pos_err);

        
        if(abs_err > hctrl->brake_threshold)
        {
            hctrl->flag_approaching = 1;
        }
        else if(abs_err < (hctrl->brake_threshold - hctrl->hyst_band))
        {
            hctrl->flag_approaching = 0;
        }

        // 更新转子目标位置
        hctrl->rotor_pos_cmd = hctrl->pos_target_out * MOTOR_GEAR_RATIO;
        hctrl->use_rot_spd   = 0.0f;      //定点模式期望转速置0
    }
    // 通信失败：状态保持不变，不修改参数

    return comm_state;
}

void GO8010_PosCtrl_SetTarget(GO8010_PosCtrl_t *hctrl, float target_rad)//设置任意角度
{
    hctrl->pos_target_out = target_rad;
}

int GO8010_PosCtrl_StayCurrentPos(GO8010_PosCtrl_t *hctrl, GO8010_Motor_t *hmotor)
{
    // 先执行一次收发，拿到最新真实转子角度 real_pos
    GO8010_Motor_SetHybrid(hmotor, hctrl->feed_torque, hctrl->use_rot_spd,
                           hctrl->rotor_pos_cmd,
                           hctrl->flag_approaching ? hctrl->k_p_far : hctrl->k_p_near,
                           hctrl->k_w);

    int comm_state = GO8010_Motor_SendRecv(hmotor,20);
    if(comm_state != 1)
    {
        return 0; //通信失败，无法获取当前位置，直接返回
    }

    //捕获此刻真实转子角度
    hctrl->real_pos = GO8010_Motor_GetPos(hmotor);

    // 核心：把控制器目标强制等于【当前实际转子角度】
    // 转子目标命令 = 当前转子角度
    hctrl->rotor_pos_cmd = hctrl->real_pos;
    // 换算负载输出轴目标
    hctrl->pos_target_out = hctrl->real_pos / MOTOR_GEAR_RATIO;

    // 下发一帧锁定指令，保持正常kp、kw刚度，锁死在当前位置
    GO8010_Motor_SetHybrid(hmotor,
    0.0f,
    0.0f,
    hctrl->rotor_pos_cmd,
    hctrl->flag_approaching ? hctrl->k_p_far : hctrl->k_p_near,
    hctrl->k_w);

    return GO8010_Motor_SendRecv(hmotor,20);
}

void GO8010_PosCtrl_Stop(GO8010_PosCtrl_t *hctrl, GO8010_Motor_t *hmotor)
{
    if(hctrl == NULL || hmotor == NULL)
        return; //这里注意！不要return 0；void函数不能带返回值！

    GO8010_Motor_Stop(hmotor);
    GO8010_Motor_SendRecv(hmotor,20);

    hctrl->pos_target_out = 0.0f;
    hctrl->rotor_pos_cmd = 0.0f;
}

uint8_t GO8010_PosCtrl_CheckArrived(GO8010_PosCtrl_t *hctrl)
{
//    static uint32_t stable_cnt = 0;

    float err = fabs(hctrl->pos_target_out - hctrl->real_pos_out);
    float spd_abs = fabs(hctrl->real_v_out);

    if (err < hctrl->pos_arrive_thr && spd_abs < hctrl->spd_arrive_thr)
    {
        if (hctrl->arrive_stable_cnt < hctrl->arrive_filter_frame)
        {
            hctrl->arrive_stable_cnt++;
        }
    }
    else
    {
        hctrl->arrive_stable_cnt = 0U;
    }

    return hctrl->arrive_stable_cnt >= hctrl->arrive_filter_frame ? 1U : 0U;
}
