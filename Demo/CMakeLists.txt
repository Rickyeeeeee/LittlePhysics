# CMakeList.txt : CMake project for LittlePhysics, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)
#if (MSVC)
#endif()
# Add source to this project's executable.
add_executable (Demo main.cpp "renderer.h" "renderer.cpp" "utils.h" "imgui_impl_opengl3.cpp" "imgui_impl_glfw.cpp" "imgui_impl_opengl3.h" "imgui_impl_glfw.h" "imgui_impl_opengl3_loader.h")
#set_target_properties(${BUILD_TARGET} PROPERTIES LINK_FLAGS "/PROFILE")

target_link_libraries(
	Demo 
	glfw
	glad
	imgui
	LittlePhysics
)