#include "encoder_odo.h"
#include "stm32h7xx_hal.h"
#include <math.h>

#define ENCODER_RADIUS 0.025  // m

#define FL_ID 1
#define FR_ID 2
#define ENCODER_ODO_CNT_PER_REV 4096
#define SQRT_2 1.4142135624
#define ENCODER_ODO_PI 3.14159265358979323846

/* 码盘通信命令（根据手册确认） */
#define ENCODER_CMD_SET_MODE  0x04u  /* 设置工作模式 */
#define ENCODER_MODE_MANUAL   0x00u  /* 手动回传模式（查询模式） */
#define ENCODER_MODE_AUTO_VALUE 0xAAu /* 自动返回编码器值 */
#define ENCODER_AUTO_RETURN_TIME_US 1000u /* 自动回传周期：1000us = 1ms = 1000Hz */
#define ENCODER_CMD_READ_CNT  0x01u  /* 读取当前计数值 */

// 码盘的安装参数
#define ENCODER_FL_POS_X 0.257
#define ENCODER_FL_POS_Y 0.065
#define ENCODER_FR_POS_X 0.257
#define ENCODER_FR_POS_Y -0.065
#define ENCODER_FL_DIR_X (SQRT_2 / 2)
#define ENCODER_FL_DIR_Y (- SQRT_2 / 2)
#define ENCODER_FR_DIR_X (- SQRT_2 / 2)
#define ENCODER_FR_DIR_Y (- SQRT_2 / 2)

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

/* 向指定 ID 的编码器发送 5 字节命令：LEN + ID + FUNC + DATA16(小端) */
static void EncoderOdo_SendCmd16(EncoderOdo *eo, uint8_t id, uint8_t func, uint16_t data)
{
    uint8_t cmd[5];
    FDCAN_TxHeaderTypeDef tx_header;

    if (eo == NULL || eo->hfdcan == NULL) {
        return;
    }

    cmd[0] = 0x05;  // 数据长度：包括 LEN、ID、FUNC、DATA
    cmd[1] = id;
    cmd[2] = func;
    cmd[3] = (uint8_t)(data & 0xFFu);
    cmd[4] = (uint8_t)((data >> 8) & 0xFFu);

    if (HAL_FDCAN_StdDefault_TxHeaderInit(&tx_header, id, 5, eo->hfdcan) != HAL_OK) {
        return;
    }

    HAL_FDCAN_Std_SendMessage(&tx_header, eo->hfdcan, cmd);
}

void EncoderOdo_Init(EncoderOdo *eo, FDCAN_HandleTypeDef *hfdcan, double iir_alpha) 
{
    eo->hfdcan = hfdcan;
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterType   = FDCAN_FILTER_MASK;
    // 原 FIFO 接收配置（保留注释）
    // filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterConfig = FDCAN_FILTER_TO_RXBUFFER;  // 改为 BUFFER 接收，同 M3508
    filter.FilterID2    = 0x7FF;   // 全匹配

    // 接收 ID=1 的码盘 -> RxBuffer0
    filter.FilterIndex = 0;
    filter.RxBufferIndex = 0;
    filter.FilterID1   = 1;
    HAL_FDCAN_ConfigFilter(hfdcan, &filter);

    // 接收 ID=2 的码盘 -> RxBuffer1
    filter.FilterIndex = 1;
    filter.RxBufferIndex = 1;
    filter.FilterID1   = 2;
    HAL_FDCAN_ConfigFilter(hfdcan, &filter);

    eo->FL_id = FL_ID;
    eo->FR_id = FR_ID;
    eo->radius = ENCODER_RADIUS;
    eo->cnt_p_rev = ENCODER_ODO_CNT_PER_REV;

    eo->FL_cur_cnt = 0;
    eo->FR_cur_cnt = 0;
    eo->FL_initialized = 0;
    eo->FR_initialized = 0;

    eo->FL_pos_x = ENCODER_FL_POS_X;   // 左前码盘相对车中心 x
    eo->FL_pos_y = ENCODER_FL_POS_Y;   // y
    eo->FL_dir_x = ENCODER_FL_DIR_X;   // 滚动方向
    eo->FL_dir_y = ENCODER_FL_DIR_Y;

    eo->FR_pos_x = ENCODER_FR_POS_X;
    eo->FR_pos_y = ENCODER_FR_POS_Y;
    eo->FR_dir_x = ENCODER_FR_DIR_X;
    eo->FR_dir_y = ENCODER_FR_DIR_Y;

    eo->abs_x = 0;
    eo->abs_y = 0;

    eo->abs_vx = 0.0;
    eo->abs_vy = 0.0;
    eo->rel_vx = 0.0;
    eo->rel_vy = 0.0;
    eo->iir_alpha = iir_alpha;

    /* 指令 0x07：设置编码器值递增方向，0x00=顺时针 */
    EncoderOdo_SendCmd(eo, eo->FL_id, 0x07, 0x00);
    EncoderOdo_SendCmd(eo, eo->FR_id, 0x07, 0x00);

    // 先设置自动回传时间 1000us
    EncoderOdo_SendCmd16(eo, eo->FL_id, 0x05, ENCODER_AUTO_RETURN_TIME_US);
    EncoderOdo_SendCmd16(eo, eo->FR_id, 0x05, ENCODER_AUTO_RETURN_TIME_US);
    // 再设置为自动返回编码器值

    EncoderOdo_SendCmd(eo, eo->FL_id, ENCODER_CMD_SET_MODE, ENCODER_MODE_AUTO_VALUE);
    EncoderOdo_SendCmd(eo, eo->FR_id, ENCODER_CMD_SET_MODE, ENCODER_MODE_AUTO_VALUE);
}

// 获取初始编码器值：软件零点（第一次收到的码盘值作为零点），不再发送清零指令
void EncoderOdo_SetBeginCnt(EncoderOdo *eo)
{
    if (eo == NULL || eo->hfdcan == NULL) {
        return;
    }

    /* 本地里程计状态同步清零 */
    eo->FL_cur_cnt = 0;
    eo->FR_cur_cnt = 0;
    eo->FL_lst_cnt = 0;
    eo->FR_lst_cnt = 0;
    eo->FL_initialized = 0;
    eo->FR_initialized = 0;
    eo->abs_x = 0.0;
    eo->abs_y = 0.0;
    eo->last_abs_x = 0.0;
    eo->last_abs_y = 0.0;
    eo->last_yaw = 0.0;
    eo->yaw_initialized = 0;

    eo->abs_vx = 0.0;
    eo->abs_vy = 0.0;
    eo->rel_vx = 0.0;
    eo->rel_vy = 0.0;
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
// 新解算：考虑码盘安装位置、旋转差分、IMU yaw
void EncoderOdo_Update(EncoderOdo *eo, double yaw, double dt)
{
    if (eo == NULL || eo->hfdcan == NULL || dt <= 0.0) {
        return;
    }

    uint8_t cnt_data[8];
    FDCAN_RxHeaderTypeDef header;
    double d_FL = 0.0;
    double d_FR = 0.0;
    int16_t delta_cnt;

    // 读取左前码盘
    if (HAL_FDCAN_GetRxMessage(eo->hfdcan, 0, &header, cnt_data) == HAL_OK) {
        if (cnt_data[0] == 7 && cnt_data[1] == 1 && cnt_data[2] == 1) {
            eo->FL_cur_cnt = (uint16_t)(cnt_data[3] | (cnt_data[4] << 8));
            if (!eo->FL_initialized) {
                eo->FL_lst_cnt = eo->FL_cur_cnt;
                eo->FL_initialized = 1;
            } else {
                delta_cnt = EncoderOdo_Cnt_Solver(eo->FL_cur_cnt, eo->FL_lst_cnt);
                d_FL = (double)delta_cnt / eo->cnt_p_rev * (2.0 * ENCODER_ODO_PI * eo->radius);
                eo->FL_lst_cnt = eo->FL_cur_cnt;
            }
        }
    }

    // 读取右前码盘
    if (HAL_FDCAN_GetRxMessage(eo->hfdcan, 1, &header, cnt_data) == HAL_OK) {
        if (cnt_data[0] == 7 && cnt_data[1] == 2 && cnt_data[2] == 1) {
            eo->FR_cur_cnt = (uint16_t)(cnt_data[3] | (cnt_data[4] << 8));
            if (!eo->FR_initialized) {
                eo->FR_lst_cnt = eo->FR_cur_cnt;
                eo->FR_initialized = 1;
            } else {
                delta_cnt = EncoderOdo_Cnt_Solver(eo->FR_cur_cnt, eo->FR_lst_cnt);
                d_FR = (double)delta_cnt / eo->cnt_p_rev * (2.0 * ENCODER_ODO_PI * eo->radius);
                eo->FR_lst_cnt = eo->FR_cur_cnt;
            }
        }
    }

    // 首次 yaw 只记录，不积分，避免启动瞬间跳变
    if (!eo->yaw_initialized) {
        eo->last_yaw = yaw;
        eo->last_abs_x = eo->abs_x;
        eo->last_abs_y = eo->abs_y;
        eo->yaw_initialized = 1;
        return;
    }

    // 由 IMU yaw 差分得到 omega
    double dyaw = yaw - eo->last_yaw;
    while (dyaw > ENCODER_ODO_PI) dyaw -= 2.0 * ENCODER_ODO_PI;
    while (dyaw < -ENCODER_ODO_PI) dyaw += 2.0 * ENCODER_ODO_PI;
    // 直接使用码盘位移增量，不先转成速度再积分
    // 旋转引起的码盘位移补偿：
    // Δs_i = ex*Δx + ey*Δy + Δψ*(-py*ex + px*ey)
    // 所以 b_i = Δs_i + Δψ*(py*ex - px*ey)
    double b_FL = d_FL + dyaw * (eo->FL_pos_y * eo->FL_dir_x - eo->FL_pos_x * eo->FL_dir_y);
    double b_FR = d_FR + dyaw * (eo->FR_pos_y * eo->FR_dir_x - eo->FR_pos_x * eo->FR_dir_y);

    double ex_FL = eo->FL_dir_x;
    double ey_FL = eo->FL_dir_y;
    double ex_FR = eo->FR_dir_x;
    double ey_FR = eo->FR_dir_y;

    double det = ex_FL * ey_FR - ey_FL * ex_FR;
    if (fabs(det) < 1e-9) {
        eo->last_yaw = yaw;
        return;
    }

    // 解算车体位移增量 Δx_body / Δy_body
    double dx_body = (b_FL * ey_FR - ey_FL * b_FR) / det;
    double dy_body = (ex_FL * b_FR - b_FL * ex_FR) / det;

    // 用积分区间中点的 yaw 做旋转，精度更高
    double mid_yaw = eo->last_yaw + 0.5 * dyaw;
    double cos_yaw = cos(mid_yaw);
    double sin_yaw = sin(mid_yaw);

    // 直接累计世界系位移
    eo->abs_x += dx_body * cos_yaw - dy_body * sin_yaw;
    eo->abs_y += dx_body * sin_yaw + dy_body * cos_yaw;

    // 由位移增量直接得到相对速度（车体系），再做低通滤波
    double rel_vx_raw = dx_body / dt;
    double rel_vy_raw = dy_body / dt;

    double alpha = eo->iir_alpha;
    eo->rel_vx = alpha * rel_vx_raw + (1.0 - alpha) * eo->rel_vx;
    eo->rel_vy = alpha * rel_vy_raw + (1.0 - alpha) * eo->rel_vy;

    // 相对速度旋转到世界系，得到绝对速度
    double cos_y = cos(yaw);
    double sin_y = sin(yaw);

    eo->abs_vx = eo->rel_vx * cos_y - eo->rel_vy * sin_y;
    eo->abs_vy = eo->rel_vx * sin_y + eo->rel_vy * cos_y;

    eo->last_abs_x = eo->abs_x;
    eo->last_abs_y = eo->abs_y;
    eo->last_yaw = yaw;
}

/* ==================== 旧解算逻辑保留注释 ====================
// // 读取码盘计数并且解算出xy的位移
// void EncoderOdo_Update(EncoderOdo *eo)
// {
//     uint8_t cnt_data[8];
//     FDCAN_RxHeaderTypeDef header;
//     if (HAL_FDCAN_StdDefault_RxHeaderInit(&header, eo->FL_id, 7, eo->hfdcan) != HAL_OK) {
//         return;
//     }
//
//     // 使用 RxBuffer 接收（同 M3508 逻辑），只检查目标 ID 对应的 Buffer
//     // 左前轮 ID=1 -> RxBuffer0
//
//     if (HAL_FDCAN_GetRxMessage(eo->hfdcan, 0, &header, cnt_data) == HAL_OK) {
//         if (cnt_data[0] == 7 && cnt_data[1] == 1 && cnt_data[2] == 1) {  // 数据长度，ID为1，指令为发送cnt
//             eo->FL_cur_cnt = (uint16_t)(cnt_data[3] | (cnt_data[4] << 8));
//             if (!eo->FL_initialized) {
//                 // 第一次收到的计数值作为零点
//                 eo->FL_lst_cnt = eo->FL_cur_cnt;
//                 eo->FL_initialized = 1;
//             } else {
//                 int16_t delta_cnt = EncoderOdo_Cnt_Solver(eo->FL_cur_cnt, eo->FL_lst_cnt);
//                 eo->abs_x += (double)delta_cnt / ENCODER_ODO_CNT_PER_REV * ENCODER_ODO_PI * SQRT_2 * eo->radius;
//                 eo->abs_y -= (double)delta_cnt / ENCODER_ODO_CNT_PER_REV * ENCODER_ODO_PI * SQRT_2 * eo->radius;
//                 eo->FL_lst_cnt = eo->FL_cur_cnt;
//             }
//         }
//     }
//
//     // 右前轮 ID=2 -> RxBuffer1
//     if (HAL_FDCAN_GetRxMessage(eo->hfdcan, 1, &header, cnt_data) == HAL_OK) {
//         if (cnt_data[0] == 7 && cnt_data[1] == 2 && cnt_data[2] == 1) {
//             eo->FR_cur_cnt = (uint16_t)(cnt_data[3] | (cnt_data[4] << 8));
//             if (!eo->FR_initialized) {
//                 // 第一次收到的计数值作为零点
//                 eo->FR_lst_cnt = eo->FR_cur_cnt;
//                 eo->FR_initialized = 1;
//             } else {
//                 int16_t delta_cnt = EncoderOdo_Cnt_Solver(eo->FR_cur_cnt, eo->FR_lst_cnt);
//                 eo->abs_x -= (double)delta_cnt / ENCODER_ODO_CNT_PER_REV * ENCODER_ODO_PI * SQRT_2 * eo->radius;
//                 eo->abs_y -= (double)delta_cnt / ENCODER_ODO_CNT_PER_REV * ENCODER_ODO_PI * SQRT_2 * eo->radius;
//                 eo->FR_lst_cnt = eo->FR_cur_cnt;
//             }
//         }
//     }
// }
*/
