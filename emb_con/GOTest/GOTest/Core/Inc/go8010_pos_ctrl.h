#ifndef __GO8010_POS_CTRL_H
#define __GO8010_POS_CTRL_H

#include "GO8010_driver.h"
#include <stdint.h>

// ===================== 可配置宏，可根据需求放到cube配置区 =====================
#define CTRL_LOOP_DELAY_MS  1U         // 控制周期，HAL_Delay(1)
// ===========================================================================

// 控制器结构体：把所有静态变量全部打包，支持多电机实例！
typedef struct
{
    // 控制参数（外部可读写）
    float k_p_far;             // 远距离Kp
    float k_p_near;            // 近距离Kp
    float k_w;
    float pos_target_out;      // 输出轴目标角度 rad
    float brake_threshold;     // 制动阈值 rad
    float hyst_band;           // 滞回带宽 rad
    float ff_mag;              // 前馈幅值（预留）
	float real_v_out;        //【新增】负载输出轴转速 rad/s

    // 内部运行状态【私有，外部尽量不要直接修改】
    float use_rot_spd;
    float feed_torque;
    float rotor_pos_cmd;
    uint8_t flag_approaching;  // 远距离逼近标志
	float real_pos_out;//当前真实负载端角度
	float real_pos;//转子端角度度
	//==== 新增到位检测参数 ====
    float pos_arrive_thr;    // 位置误差阈值（负载轴 rad）
    float spd_arrive_thr;    // 静止转速阈值（负载轴 rad/s）
    uint32_t arrive_filter_frame; // 防抖连续帧数
	uint32_t arrive_stable_cnt;//防抖计数器

} GO8010_PosCtrl_t;

/**
 * @brief  定点控制器初始化，填充默认参数
 * @param  hctrl: 控制器实例指针
 * @retval 无
 */
void GO8010_PosCtrl_Init(GO8010_PosCtrl_t *hctrl);

/**
 * @brief  执行一次力位混合位置控制循环
 * @param  hctrl: 控制器实例
 * @param  hmotor: GO8010电机底层实例
 * @retval 通信状态 1=正常，0=通讯失败
 * @note   需要周期性调用（1ms一次，和原while逻辑一致）
 */
int GO8010_PosCtrl_Run(GO8010_PosCtrl_t *hctrl, GO8010_Motor_t *hmotor);

void GO8010_PosCtrl_SetTarget180(GO8010_PosCtrl_t *hctrl);
void GO8010_PosCtrl_SetTarget90(GO8010_PosCtrl_t *hctrl);
/**
 * @brief  设置输出轴目标角度 rad
 */
void GO8010_PosCtrl_SetTarget(GO8010_PosCtrl_t *hctrl, float target_rad);

/**
 * @brief FOC模式下停机，锁定保持在电机当前真实位置
 * @note 不退出FOC模式，后续直接SetTarget即可继续运动，无需重新使能
 * @retval 1通信成功，0通信失败
 */
int GO8010_PosCtrl_StayCurrentPos(GO8010_PosCtrl_t *hctrl, GO8010_Motor_t *hmotor);//急停函数
void GO8010_PosCtrl_Stop(GO8010_PosCtrl_t *hctrl, GO8010_Motor_t *hmotor);//失能函数
uint8_t GO8010_PosCtrl_CheckArrived(GO8010_PosCtrl_t *hctrl);

#endif
