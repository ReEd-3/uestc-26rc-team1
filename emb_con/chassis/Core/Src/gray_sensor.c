#include "gray_sensor.h"
#include "main.h"

static uint16_t gray_raw[GRAY_NUM];
static uint16_t gray_filtered[GRAY_NUM];
static float last_line_error = 0;   // 用于保存上一次的线偏差

void Gray_Init(void) {
    // 初始化ADC或GPIO，根据传感器类型
}

void Gray_Read(void) {
    // 读取传感器，例如数字量：
    for (int i = 0; i < GRAY_NUM; i++) {
        // gray_raw[i] = HAL_GPIO_ReadPin(...) ? 1 : 0;
        // 模拟量则读取ADC
    }
    // 简单滤波
    for (int i = 0; i < GRAY_NUM; i++) {
        gray_filtered[i] = (gray_filtered[i] * 3 + gray_raw[i]) / 4;
    }
}

float Gray_GetLineError(void) {
    uint8_t on_line[GRAY_NUM] = {0};

    // 1. 二值化：根据阈值判断
    for (int i = 0; i < GRAY_NUM; i++) {
        on_line[i] = (gray_filtered[i] > THRESHOLD) ? 1 : 0;  // 根据极性调整
    }

    // 2. 寻找左右边缘
    int left = -1, right = -1;
    for (int i = 0; i < GRAY_NUM; i++) {
        if (on_line[i]) {
            left = i;
            break;
        }
    }
    for (int i = GRAY_NUM - 1; i >= 0; i--) {
        if (on_line[i]) {
            right = i;
            break;
        }
    }

    // 3. 如果完全没有检测到线，返回保持上次偏差或特殊值
    if (left == -1 || right == -1) {
        return last_line_error;  // 或返回0，但需在外部处理丢线
    }

    // 4. 计算线中心对应的传感器索引（浮点）
    float center_index = (left + right) / 2.0f;

    // 5. 将传感器索引转换为物理位置，再归一化到 -1..1
    // 假设传感器间距为1个单位，物理中心在索引3.5处（取决于你的 pos 定义）
    // 简单做法：直接用 (center_index - 3.5) / 3.5 得到归一化偏差
    float error = (center_index - 3.5f) / 3.5f;   // 假设中心在3.5，范围[-1,1]

    // 可选：限制在合理范围
    if (error > 1.0f) error = 1.0f;
    if (error < -1.0f) error = -1.0f;

    last_line_error = error;  // 保存本次偏差
    return error;
}
