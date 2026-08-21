add_library(chassis STATIC
    chassis.c
)

target_include_directories(chassis PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(chassis PUBLIC
    m3508
    macnum
    encoder_odo
    basic_dependencies
    basic_protocol
    app
)
