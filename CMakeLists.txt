cmake_minimum_required(VERSION 2.6.2)
project(corgui)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -pthread -march=native")
set(CORGUI_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/")
set(IMGUI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/imgui/")
	
add_library(imgui STATIC
	${IMGUI_DIR}/imgui.cpp
	${IMGUI_DIR}/imgui_draw.cpp
	${IMGUI_DIR}/examples/opengl3_example/imgui_impl_glfw_gl3.cpp
	)

target_link_libraries(imgui
  gl3w
  dl
  )
target_include_directories(imgui PUBLIC ${IMGUI_DIR})

add_subdirectory(src)
add_subdirectory(examples)
