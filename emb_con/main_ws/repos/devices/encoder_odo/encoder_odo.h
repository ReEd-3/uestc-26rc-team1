#ifndef ENCODER_ODO_H
#define ENCODER_ODO_H

#include "fdcan_std.h"
#include "stm32h7xx_hal.h"
#include "iir.h"

// 码盘结构体
typedef struct {
    // can总线
    FDCAN_HandleTypeDef *hfdcan;
    uint8_t FL_id;
    uint8_t FR_id;

    // 两个编码器位移
    uint16_t cnt_p_rev;  // 编码器一圈的计数
    double radius;  // 轮子半径

    // 当前计数
    uint16_t FL_cur_cnt;
    uint16_t FR_cur_cnt;

    // 上次计数
    uint16_t FL_lst_cnt;
    uint16_t FR_lst_cnt;

    // 首次接收零点标志：第一次收到的计数值作为零点
    uint8_t FL_initialized;
    uint8_t FR_initialized;

    double FR_x;  // 码盘右前原始位移,m
    double FL_x;  // 码盘左前原始位移

    double abs_x;  // 码盘绝对位移
    double abs_y;  // 

    double abs_vx;  // 世界系绝对速度
    double abs_vy;

    double rel_vx;  // 自身坐标系速度
    double rel_vy;

    double iir_alpha;

    // 左前码盘
    double FL_pos_x, FL_pos_y;
    double FL_dir_x, FL_dir_y;

    // 右前码盘
    double FR_pos_x, FR_pos_y;
    double FR_dir_x, FR_dir_y;

    // 上一周期位置，用于差分/滤波
    double last_abs_x, last_abs_y;
    double last_yaw;
    uint8_t yaw_initialized;
} EncoderOdo;

void EncoderOdo_Init(EncoderOdo *eo, FDCAN_HandleTypeDef *hfdcan, double iir_alpha);

// 计算初始位置编码器值 
void EncoderOdo_SetBeginCnt(EncoderOdo *eo);

// 计算编码器改变量
int16_t EncoderOdo_Cnt_Solver(uint16_t cur_cnt, uint16_t lst_cnt);

// 读取码盘计数并且解算出xy的位移
void EncoderOdo_Update(EncoderOdo *eo, double yaw, double dt);

#endif