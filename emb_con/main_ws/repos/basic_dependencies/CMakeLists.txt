add_library(basic_dependencies STATIC
    iir.c
    pid.c
) 

target_include_directories(basic_dependencies PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)