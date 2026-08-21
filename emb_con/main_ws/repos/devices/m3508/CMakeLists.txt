add_library(m3508 STATIC
    m3508_driver.c
)
target_include_directories(m3508 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}                # 让 "m3508_driver.h" 可见
)
target_link_libraries(m3508 PUBLIC 
    stm32cubemx
    basic_dependencies
    basic_protocol
)