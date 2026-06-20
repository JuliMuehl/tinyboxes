cmake_minimum_required(VERSION 4.3.0)
project(tinyboxes)
set(CMAKE_EXPORT_COMPILE_COMMANDS on)

add_library(tinyboxes src/TinyBoxes.cpp src/Collision.cpp)
include_directories(include)

add_subdirectory(demos)
