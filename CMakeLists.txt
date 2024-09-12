cmake_minimum_required(VERSION 3.10)

project(Switchable-Edge-Search)

# 设置编译器选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Ofast")

# 添加源文件
file(GLOB_RECURSE SRC_FILES
    src/types.h
    src/graph/*.h
    src/graph/*.cpp
    src/ADG/*.h
    src/ADG/*.cpp
    src/Algorithm/*.h
    src/Algorithm/*.cpp
    src/simulate.cpp
)

# 添加可执行文件
add_executable(a.out ${SRC_FILES})
