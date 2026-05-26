cmake_minimum_required(VERSION 3.8.2)
project(opengl_lesson_09)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(Qt5 COMPONENTS Core Widgets OpenGL REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(opengl_lesson_09
  main.cpp
  obj_parser.cpp
  obj_parser.h
  widget_gl.cpp
  widget_gl.h
  resources.qrc
)

target_include_directories(opengl_lesson_09 PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${HELPER_OPENGL_DIR}
)

target_link_libraries(opengl_lesson_09
  helper_opengl
  Eigen3::Eigen
  Qt5::Core
  Qt5::Widgets
  Qt5::OpenGL
)
