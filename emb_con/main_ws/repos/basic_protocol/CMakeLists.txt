add_library(basic_protocol STATIC
    fdcan_std.c
) 

target_include_directories(basic_protocol PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(basic_protocol PUBLIC
    stm32cubemx
)