cmake_minimum_required(VERSION 3.8.2)

project(inverse_kinematics_sim_app)

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
find_package(Qt5 COMPONENTS Network Widgets SerialPort REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(inverse_kinematics_sim_app 
	             main.cpp
	             romi_robot_server.h 
	             romi_robot_server.cpp
	             motion_control_thread.h
	             motion_control_thread.cpp)
target_link_libraries(inverse_kinematics_sim_app 
                      Qt5::Network 
                      Eigen3::Eigen 
                      robot_sim_models)
