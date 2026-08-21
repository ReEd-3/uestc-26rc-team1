add_library(encoder_odo STATIC
    encoder_odo.c
)

target_include_directories(encoder_odo PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(encoder_odo PUBLIC stm32cubemx basic_protocol)