cmake_minimum_required(VERSION 3.8.2)

project(inverse_kinematics_viewer)

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

add_executable(inverse_kinematics_viewer
	             path_view_widget.h 
	             path_view_widget.cpp
	             main_dlg.ui
	             main_dlg.h
	             main_dlg.cpp
							 comms_thread.h
							 comms_thread.cpp
               main.cpp
               resources.qrc)
target_include_directories(inverse_kinematics_viewer PRIVATE .)
target_link_libraries(inverse_kinematics_viewer 
                      Qt5::Widgets 
                      Qt5::SerialPort 
                      Qt5::Network 
                      helper_opengl)
