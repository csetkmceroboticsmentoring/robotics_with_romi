cmake_minimum_required(VERSION 3.8.2)

project(inverse_kinematics_app_i2c)

# Find includes in the build directories
#set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Turn on automatic invocation of the MOC, UIC & RCC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 14)

# Add a compiler flag
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# Find the QtWidgets library
find_package(Qt5 COMPONENTS Network Widgets REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(inverse_kinematics_app_i2c 
	             main.cpp
	             romi_robot_server.h 
	             romi_robot_server.cpp
	             motion_control_thread.h
	             motion_control_thread.cpp
	             i2c_slave.h
	             i2c_slave.cpp
	             cmd_response.h)
target_link_libraries(inverse_kinematics_app_i2c Qt5::Network Qt5::Core Eigen3::Eigen)
