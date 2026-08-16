#include "uart_io.hpp"
#include "stm32f1xx_hal.h"
#include <cstdint>

namespace {

constexpr uint16_t kFrameSize = 16;

Serial_Test::Uart_IO *g_uart_io = nullptr;

}

extern "C" {

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (g_uart_io != nullptr && g_uart_io->getHandle() != nullptr && huart->Instance == g_uart_io->getHandle()->Instance) {
        g_uart_io->onRxCallback(Size);
    }
}

}

namespace Serial_Test {

void Uart_IO::start(UART_HandleTypeDef *_huart) {  // 开启接收
    io_huart = _huart;
    g_uart_io = this;
    HAL_UARTEx_ReceiveToIdle_IT(io_huart, rx_buffer.data(), rx_buffer.size());
}

void Uart_IO::onRxCallback(uint16_t size) {
    if (size == kFrameSize) {
        rx_ready = true;
    }
}

void Uart_IO::process() {
    if (rx_ready == true) {
        rx_ready = false;
        HAL_UART_Transmit(io_huart, rx_buffer.data(), kFrameSize, 0xFF);
        HAL_UARTEx_ReceiveToIdle_IT(io_huart, rx_buffer.data(), rx_buffer.size());
    }
}

}

