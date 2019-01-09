cmake_minimum_required(VERSION 3.10)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

set(RUN_DIR ${PROJECT_SOURCE_DIR}/bin)
find_package(OpenGL)

set(IMGUI_SOURCES
        "include/imgui/imgui.cpp"
        "include/imgui/imgui_demo.cpp"
        "include/imgui/imgui_widgets.cpp"
        "include/imgui/imgui_impl_glfw.cpp"
        "include/imgui/imgui_impl_opengl3.cpp"
        "include/imgui/imgui_draw.cpp")

set(IMGUI_HEADERS
        "include/imgui/imgui.h"
        "include/imgui/imconfig.h"
        "include/imgui/imgui_impl_opengl3.h"
        "include/imgui/imgui_impl_glfw.h"
        "include/imgui/imstb_rectpack.h"
        "include/imgui/imstb_textedit.h"
        "include/imgui/imstb_truetype.h")

find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW REQUIRED glfw3)
set(COMMON_LIBS sb7 ${GLFW_LIBRARIES} GL rt dl OpenMeshCore OpenMeshTools)

link_libraries(stdc++fs)

add_library(sb7 include/sb7.cpp include/sb7shader.cpp include/gl3w.c)

add_executable(subdivision_app src/App.cpp
        src/LabelSubdivision.cpp src/LabelSubdivision.h
        src/Object.cpp src/Object.h
        ${IMGUI_SOURCES} ${IMGUI_HEADERS})

target_link_libraries(subdivision_app ${COMMON_LIBS} )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_LINUX -std=c++11")

include_directories(include)
