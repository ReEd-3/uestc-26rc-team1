add_library(encoder_odo STATIC
    encoder_odo.c
)

target_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(encoder_odo PUBLIC stm32cubemx m3508)