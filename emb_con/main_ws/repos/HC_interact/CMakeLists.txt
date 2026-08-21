add_library(hc_interact STATIC
    uart_interact.c
)

target_include_directories(hc_interact PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 纯协议层不依赖 HAL/FreeRTOS，因此这里不需要链接 stm32cubemx
