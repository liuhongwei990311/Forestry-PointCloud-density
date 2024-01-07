cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(DenseExample)

# 设置PCL库
find_package(PCL 1.8 REQUIRED )

# 添加头文件路径
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# 添加可执行文件
add_executable(DenseExample square.cpp)

# 链接PCL库
target_link_libraries(DenseExample ${PCL_LIBRARIES})

