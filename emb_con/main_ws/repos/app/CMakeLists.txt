add_library(app STATIC
    app.c
)
# 添加头文件路径
target_include_directories(app PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(app PUBLIC
    stm32cubemx
    chassis
    task1
    hc_interact
    m3508
    encoder_odo
    basic_dependencies
    basic_protocol
)
