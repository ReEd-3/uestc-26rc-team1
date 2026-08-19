#ifndef ENCODER_ODO_H
#define ENCODER_ODO_H

#include "fdcan_std.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_fdcan.h"

#define ENCODER_RADIUS 0.0

#define FL_ID 1
#define FR_ID 2

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

    double FR_x;  // 码盘右前原始位移,m
    double FL_x;  // 码盘左前原始位移

    double abs_x;  // 码盘绝对位移
    double abs_y;  // 
    double omega;  // 码盘当前旋转角度
} EncoderOdo;

void EncoderOdo_Init(EncoderOdo *eo, FDCAN_HandleTypeDef *hfdcan) 
{

}
 
void EncoderOdo_Update(EncoderOdo *eo, uint16_t FL_cnt, uint16_t FR_cnt)
{
    uint8_t cnt_data[7];
    FDCAN_RxHeaderTypeDef header;
    if (HAL_FDCAN_StdDefault_RxHeaderInit(&header, eo->FL_id, 7, eo->hfdcan) != HAL_OK) {
        return;
    }
    if (HAL_FDCAN_Std_ReceiveMessage(header, eo->hfdcan, 0, cnt_data) != HAL_OK) { 
        return;
    } 
    // 读到的是右前轮，更新数据
    if (cnt_data[0] == 7 && cnt_data[1] == 1 && cnt_data[2] == 1) {  // 数据长度，ID为1，指令为发送cnt
        eo->FL_lst_cnt = eo->FL_cur_cnt;
        eo->FL_cur_cnt = (uint16_t)(cnt_data[0] & (cnt_data[1] << 8));
    }
    // 左前轮同理
    else if (cnt_data[0] == 7 && cnt_data[1] == 2 && cnt_data[2] == 1) {
        eo->FR_lst_cnt = eo->FR_cur_cnt;
        eo->FR_cur_cnt = (uint16_t)(cnt_data[0] & (cnt_data[1] << 8));
    }

}

#endif