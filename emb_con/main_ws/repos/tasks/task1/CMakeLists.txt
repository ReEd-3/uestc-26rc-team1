add_library(task1 STATIC
    chassis_task_1.c
) 

target_include_directories(task1 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(task1 PUBLIC
    stm32cubemx
    chassis
    basic_dependencies
    common_tasks
)