#include "vision.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

VisionInfo_t vision = {0};

extern uint8_t uart3_rx_buf[256];
extern uint16_t uart3_rx_len;

void Vision_Init(void) {
    vision.updated = false;
    vision.slope_finished = false;
}

// 协议示例：$VISION,R,120,80,5.0,0.35,1\r\n
// 字段：颜色,x,y,角度(度),距离(米),上坡结束标志(0/1)
void Vision_Process(void) {
    if (uart3_rx_len == 0) return;

    if (uart3_rx_buf[0] != '$') {
        uart3_rx_len = 0;
        return;
    }

    char color;
    int x, y;
    float angle, distance;
    int slope_flag = 0;

    int parsed = sscanf((char*)uart3_rx_buf, "$VISION,%c,%d,%d,%f,%f,%d",
                        &color, &x, &y, &angle, &distance, &slope_flag);

    if (parsed == 6) {
        vision.color = color;
        vision.x = x;
        vision.y = y;
        vision.angle = angle;
        vision.distance = distance;
        vision.slope_finished = (slope_flag == 1);
        vision.updated = true;
        vision.timestamp = HAL_GetTick();
    }

    uart3_rx_len = 0;
}
