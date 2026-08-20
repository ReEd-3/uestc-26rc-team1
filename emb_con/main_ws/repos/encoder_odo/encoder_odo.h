#ifndef ENCODER_ODO_H
#define ENCODER_ODO_H

#include "fdcan_std.h"
#include "stm32h7xx_hal.h"


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
} EncoderOdo;

void EncoderOdo_Init(EncoderOdo *eo, FDCAN_HandleTypeDef *hfdcan);

// 计算初始位置编码器值 
void EncoderOdo_SetBeginCnt(EncoderOdo *eo);

// 计算编码器改变量
int16_t EncoderOdo_Cnt_Solver(uint16_t cur_cnt, uint16_t lst_cnt);

// 读取码盘计数并且解算出xy的位移
void EncoderOdo_Update(EncoderOdo *eo);

#endif