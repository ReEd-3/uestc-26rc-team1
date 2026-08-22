# 添加静态库
add_library(trajectory STATIC
    beziertrajplan.c
    cubictrajplan.c
)
# 添加头文件路径
target_include_directories(trajectory PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)