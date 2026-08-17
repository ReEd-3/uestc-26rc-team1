#ifndef IIR_H
#define IIR_H

#include "stm32h7xx_hal.h"

// 滤波器句柄
typedef struct {
    int16_t raw_data;       // 原始数据（输入）
    int16_t filtered_data;  // 滤波后数据（输出）
    double filter_status;   // 内部状态：上一拍滤波值
    double filter_alpha;    // 滤波系数（0~1，1=直通不滤波）
} Int16_IIR;

void Int16_IIRFilter_Init(Int16_IIR *int16_iir, double alpha);
int16_t Int16_IIRFilter_Update(Int16_IIR *int16_iir, int16_t raw_data);

#endif
