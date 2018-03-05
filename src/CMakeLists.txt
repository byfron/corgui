add_library(gl3w gl3w.c GL/gl3w.h)
target_include_directories(gl3w PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(corgui corgui.hpp corgui.cpp ${IMGUI_DIR}/imgui.h ${IMGUI_DIR}/imgui.cpp)
target_link_libraries(corgui gl3w dl)
target_include_directories(corgui PUBLIC ${IMGUI_DIR})
