cmake_minimum_required(VERSION 3.8.2)

project(robot_sim_models)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Create library
add_library(robot_sim_models STATIC
            pid_control.h
            pid_control.cpp
            wheel_with_encoder.h)

# This is a header-only for wheel_with_encoder, but we include pid_control implementation
target_include_directories(robot_sim_models PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

