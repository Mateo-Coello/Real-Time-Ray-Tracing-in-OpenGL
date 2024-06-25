cmake_minimum_required(VERSION 3.27.0)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED TRUE)

project(raytracing VERSION 0.1.0)
find_package(glfw3 REQUIRED)

file(GLOB_RECURSE SRC_FILES src/*.c)
add_executable(raytracing main.c ${SRC_FILES})

# target_compile_options(raytracing PUBLIC -O3)
target_link_libraries(raytracing PUBLIC GL dl glfw m)
