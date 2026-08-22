add_library(yis506 STATIC
    yis506.c
)

target_include_directories(yis506 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(yis506 PUBLIC
    stm32cubemx
)