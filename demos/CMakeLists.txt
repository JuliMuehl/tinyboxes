cmake_minimum_required(VERSION 4.3.0)
project(tinyboxes)

find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
add_executable(WreckingBallDemo WreckingBall.cpp)
include_directories(.)
target_link_libraries(WreckingBallDemo PRIVATE tinyboxes glfw GLEW OpenGL)