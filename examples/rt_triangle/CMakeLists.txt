cmake_minimum_required(VERSION 3.27.0)
project(raytracing VERSION 0.1.0)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED TRUE)

# Collect source files
file(GLOB_RECURSE SRC_FILES src/*.c)

# Define the executable
add_executable(raytracing ${SRC_FILES})

# Include directories
target_include_directories(raytracing PUBLIC ${CMAKE_SOURCE_DIR}/include)

# Link GLFW and other libraries (platform specific)
if (UNIX)
    # Linux specific libraries
    target_link_libraries(raytracing PUBLIC GL dl glfw m)
elseif (WIN32)
    # Windows specific libraries (assuming MinGW-w64)
    target_link_libraries(raytracing PUBLIC ${CMAKE_SOURCE_DIR}/lib/libglfw3.a)
endif()
