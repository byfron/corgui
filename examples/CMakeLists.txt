add_executable(corgui_example main.cpp)
target_link_libraries(corgui_example corgui imgui glfw)

target_include_directories(corgui_example PUBLIC ${CORGUI_INCLUDE_DIR})
