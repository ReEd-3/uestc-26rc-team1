# 添加静态库
add_library(macnum STATIC
    macnum.c
)
# 添加头文件路径
target_include_directories(macnum PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)