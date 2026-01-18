cmake_minimum_required(VERSION 3.8.2)

project(forward_kinematics_sim)

# Find includes in the build directories
#set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Turn on automatic invocation of the MOC, UIC & RCC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)

# Add a compiler flag
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# Find the QtWidgets library
find_package(Qt5 COMPONENTS Core Widgets Gamepad REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(forward_kinematics_sim
	             path_view_widget.h 
	             path_view_widget.cpp
	             main_dlg.ui
	             main_dlg.h
	             main_dlg.cpp
               main.cpp
               resources.qrc)
target_include_directories(forward_kinematics_sim PRIVATE .)
target_link_libraries(forward_kinematics_sim 
                      Qt5::Core 
                      Qt5::Widgets 
                      Qt5::Gamepad 
                      helper_opengl 
                      virtual_controller 
                      robot_sim_models)
