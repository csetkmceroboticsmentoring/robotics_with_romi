cmake_minimum_required(VERSION 3.10)
project(romi_gazebo_plugins)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
# Note: Gazebo Fortress uses these versions
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Source files for robot models (header-only or need implementation)
set(ROBOT_MODELS_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/pid_control.cpp
)

# Plugin source files
set(PLUGIN_SOURCES
  romi_encoder_plugin.cpp
  ${ROBOT_MODELS_SOURCES}
)

# Create shared library for the plugin
add_library(romi_encoder_plugin SHARED ${PLUGIN_SOURCES})

target_link_libraries(romi_encoder_plugin
  ignition-gazebo6::core
  ignition-transport11::core
  ignition-msgs8::ignition-msgs8
  ignition-math6::ignition-math6
  ignition-plugin1::loader
)

# Install plugin
install(TARGETS romi_encoder_plugin
  LIBRARY DESTINATION lib
)

# Install plugin metadata (if needed)
# install(FILES plugin.xml DESTINATION share/romi_gazebo_plugins)

