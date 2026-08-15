#include "iir.h"

// 初始化低通滤波器参数
void Int16_IIRFilter_Init(Int16_IIR *int16_iir, double alpha) {
    int16_iir->raw_data = 0;
    int16_iir->filtered_data = 0;
    int16_iir->filter_status = 0;
    int16_iir->filter_alpha = alpha;
}

// 一阶低通滤波：filtered += alpha * (raw - filtered)
// 内部状态用 double 累加，避免 int16 每拍舍入导致状态卡死
int16_t Int16_IIRFilter_Update(Int16_IIR *int16_iir, int16_t raw_data) {
    int16_iir->raw_data = raw_data;
    int16_iir->filter_status += int16_iir->filter_alpha * ((double)raw_data - int16_iir->filter_status);
    int16_iir->filtered_data = (int16_t)int16_iir->filter_status;
    return int16_iir->filtered_data;
}