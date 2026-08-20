#include "encoder_odo.h"
#include "stm32h7xx_hal.h"

#define ENCODER_RADIUS 0.025  // m

#define FL_ID 1
#define FR_ID 2
#define ENCODER_ODO_CNT_PER_REV 4096
#define SQRT_2 1.4142135624
#define ENCODER_ODO_PI 3.14159265358979323846

/* 向指定 ID 的编码器发送 4 字节命令：LEN + ID + FUNC + DATA */
static void EncoderOdo_SendCmd(EncoderOdo *eo, uint8_t id, uint8_t func, uint8_t data)
{
    uint8_t cmd[4];
    FDCAN_TxHeaderTypeDef tx_header;

    if (eo == NULL || eo->hfdcan == NULL) {
        return;
    }

    cmd[0] = 0x04;  // 数据长度：包括 LEN、ID、FUNC、DATA
    cmd[1] = id;
    cmd[2] = func;
    cmd[3] = data;

    if (HAL_FDCAN_StdDefault_TxHeaderInit(&tx_header, id, 4, eo->hfdcan) != HAL_OK) {
        return;
    }

    HAL_FDCAN_Std_SendMessage(&tx_header, eo->hfdcan, cmd);
}

void EncoderOdo_Init(EncoderOdo *eo, FDCAN_HandleTypeDef *hfdcan) 
{
    eo->hfdcan = hfdcan;
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID2    = 0x7FF;   // 全匹配

    // 接收 ID=1 的码盘
    filter.FilterIndex = 0;
    filter.FilterID1   = 1;
    HAL_FDCAN_ConfigFilter(hfdcan, &filter);

    // 接收 ID=2 的码盘
    filter.FilterIndex = 1;
    filter.FilterID1   = 2;
    HAL_FDCAN_ConfigFilter(hfdcan, &filter);

    eo->FL_id = FL_ID;
    eo->FR_id = FR_ID;
    eo->radius = ENCODER_RADIUS;
    eo->cnt_p_rev = ENCODER_ODO_CNT_PER_REV;

    eo->FL_cur_cnt = 0;
    eo->FR_cur_cnt = 0;

    eo->abs_x = 0;
    eo->abs_y = 0;

    /* 指令 0x07：设置编码器值递增方向，0x00=顺时针 */
    EncoderOdo_SendCmd(eo, eo->FL_id, 0x07, 0x00);
    EncoderOdo_SendCmd(eo, eo->FR_id, 0x07, 0x00);
}

// 获取初始编码器值：发送指令 0x06 将当前编码器值清零
void EncoderOdo_SetBeginCnt(EncoderOdo *eo)
{
    if (eo == NULL || eo->hfdcan == NULL) {
        return;
    }

    /* 指令 0x06：设置当前位置值为零点，设置后当前编码器值为 0 */
    EncoderOdo_SendCmd(eo, eo->FL_id, 0x06, 0x00);
    EncoderOdo_SendCmd(eo, eo->FR_id, 0x06, 0x00);

    /* 本地里程计状态同步清零 */
    eo->FL_cur_cnt = 0;
    eo->FR_cur_cnt = 0;
    eo->FL_lst_cnt = 0;
    eo->FR_lst_cnt = 0;
    eo->abs_x = 0.0;
    eo->abs_y = 0.0;
}

// 计算编码器改变量,处理跨圈
int16_t EncoderOdo_Cnt_Solver(uint16_t cur_cnt, uint16_t lst_cnt) 
{
    int16_t ans = cur_cnt - lst_cnt;
    if (ans > ENCODER_ODO_CNT_PER_REV / 2) {
        ans -= ENCODER_ODO_CNT_PER_REV;
    }
    else if (ans < -ENCODER_ODO_CNT_PER_REV / 2) {
        ans += ENCODER_ODO_CNT_PER_REV;
    }
    return ans;
}

// 读取码盘计数并且解算出xy的位移
void EncoderOdo_Update(EncoderOdo *eo)
{
    uint8_t cnt_data[7];
    FDCAN_RxHeaderTypeDef header;
    if (HAL_FDCAN_StdDefault_RxHeaderInit(&header, eo->FL_id, 7, eo->hfdcan) != HAL_OK) {
        return;
    }
    while (HAL_FDCAN_Std_ReceiveMessage(&header, eo->hfdcan, 0, cnt_data) == HAL_OK) { 
        // 读到的是左前轮，更新数据
        if (cnt_data[0] == 7 && cnt_data[1] == 1 && cnt_data[2] == 1) {  // 数据长度，ID为1，指令为发送cnt
            eo->FL_lst_cnt = eo->FL_cur_cnt;
            eo->FL_cur_cnt = (uint16_t)(cnt_data[3] | (cnt_data[4] << 8));
            int16_t delta_cnt = EncoderOdo_Cnt_Solver(eo->FL_cur_cnt, eo->FL_lst_cnt);
            eo->abs_x += (double)delta_cnt / ENCODER_ODO_CNT_PER_REV * ENCODER_ODO_PI * SQRT_2 * eo->radius;
            eo->abs_y -= (double)delta_cnt / ENCODER_ODO_CNT_PER_REV * ENCODER_ODO_PI * SQRT_2 * eo->radius;
        }
        // 右前轮同理
        else if (cnt_data[0] == 7 && cnt_data[1] == 2 && cnt_data[2] == 1) {
            eo->FR_lst_cnt = eo->FR_cur_cnt;
            eo->FR_cur_cnt = (uint16_t)(cnt_data[3] | (cnt_data[4] << 8));
            int16_t delta_cnt = EncoderOdo_Cnt_Solver(eo->FR_cur_cnt, eo->FR_lst_cnt);
            eo->abs_x -= (double)delta_cnt / ENCODER_ODO_CNT_PER_REV * ENCODER_ODO_PI * SQRT_2 * eo->radius;
            eo->abs_y -= (double)delta_cnt / ENCODER_ODO_CNT_PER_REV * ENCODER_ODO_PI * SQRT_2 * eo->radius;
        }
    }
}