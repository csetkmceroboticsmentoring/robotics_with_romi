cmake_minimum_required(VERSION 3.8.2)
project(opengl_tutorial LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# Sibling directory: qt_cpp/helper_opengl
set(HELPER_OPENGL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../helper_opengl CACHE INTERNAL "")

find_package(Qt5 COMPONENTS Core Widgets OpenGL REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Build the library lessons link against (must be before lesson targets).
add_subdirectory(${HELPER_OPENGL_DIR} ${CMAKE_BINARY_DIR}/helper_opengl)

add_subdirectory(01_hello_triangle)
add_subdirectory(02_vertex_colors)
add_subdirectory(03_uniform_transform)
add_subdirectory(04_indexed_quad)
add_subdirectory(05_texture_quad)
add_subdirectory(06_dynamic_line_strip)
add_subdirectory(07_mini_path_viewer)
add_subdirectory(08_rotating_triangle_3d)
add_subdirectory(09_obj_viewer)
