#ifndef UART_IO_HPP_
#define UART_IO_HPP_

#include <vector>
#include "stm32f1xx_hal.h"

namespace Serial_Test {

class Uart_IO {
public:  // 函数API
    void start(UART_HandleTypeDef *_huart);  // 启动函数
    void onRxCallback(uint16_t size);  // 回调函数
    void process();  // 主循环调用
    const UART_HandleTypeDef *getHandle() const { return io_huart; }
private:  // 私有成员
    UART_HandleTypeDef *io_huart = nullptr;
    std::vector<uint8_t> rx_buffer = std::vector<uint8_t>(32);
    volatile bool rx_ready = false;
};

}

#endif