cmake_minimum_required(VERSION 3.8.2)

project(helper_opengl)

set(CMAKE_CXX_STANDARD 14)

# Add a compiler flag
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# Find the QtWidgets library
find_package(Qt5 REQUIRED COMPONENTS OpenGL)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_library(helper_opengl
            buffer.h 
            program.h 
            program.cpp 
            texture.h 
            texture.cpp)
target_link_libraries(helper_opengl 
                      Eigen3::Eigen 
                      Qt5::OpenGL)
