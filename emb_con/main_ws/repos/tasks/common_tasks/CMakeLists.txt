add_library(common_tasks INTERFACE) 

target_include_directories(common_tasks INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(common_tasks INTERFACE
    stm32cubemx
    chassis
    basic_dependencies
    trajectory
)