cmake_minimum_required(VERSION 3.8.2)
project(opengl_lesson_05)

set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(Qt5 COMPONENTS Core Widgets OpenGL REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(opengl_lesson_05 main.cpp)
target_include_directories(opengl_lesson_05 PRIVATE ${HELPER_OPENGL_DIR})
target_link_libraries(opengl_lesson_05
                      helper_opengl
                      Eigen3::Eigen
                      Qt5::Core
                      Qt5::Widgets
                      Qt5::OpenGL)

# Copy tkmce.jpeg next to the executable when present (run from build dir).
set(TKMCE_JPEG ${CMAKE_CURRENT_SOURCE_DIR}/tkmce.jpeg)
if(EXISTS ${TKMCE_JPEG})
  add_custom_command(TARGET opengl_lesson_05 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      ${TKMCE_JPEG}
      $<TARGET_FILE_DIR:opengl_lesson_05>/tkmce.jpeg
    COMMENT "Copy tkmce.jpeg for opengl_lesson_05")
endif()
