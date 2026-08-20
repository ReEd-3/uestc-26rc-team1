add_library(chassis STATIC
    chassis.c
    chassis_task_1.c
    uart_interact.c
    interact_cmds.c
)

target_include_directories(chassis PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(chassis PUBLIC
    m3508
    macnum
    encoder_odo
)