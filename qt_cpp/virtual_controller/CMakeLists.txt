cmake_minimum_required(VERSION 3.8.2)

project(virtual_controller)

# Turn on automatic invocation of the MOC
set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_STANDARD 17)

# Find the QtWidgets library
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

# Create library
add_library(virtual_controller STATIC
            joystick_widget.h
            joystick_widget.cpp
            control_widget.h
            control_widget.cpp)

target_include_directories(virtual_controller 
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(virtual_controller 
                      PUBLIC 
                      Qt5::Core 
                      Qt5::Widgets)

add_executable(test_joystick test_joystick.cpp)
target_link_libraries(test_joystick virtual_controller)
