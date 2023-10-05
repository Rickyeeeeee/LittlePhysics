# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.
#
cmake_minimum_required (VERSION 3.8)
set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
project ("LittlePhysics")


# Include sub-projects.
add_subdirectory ("glfw")
add_subdirectory ("glad")
add_subdirectory ("imgui/imgui")
#add_subdirectory ("LittlePhysicsEngine")
add_subdirectory ("src")
add_subdirectory ("Demo")
if (WIN32)
    target_compile_definitions(LittlePhysics PUBLIC LP_PLATFORM_WINDOWS)
endif (WIN32)

#set_target_properties(Demo PROPERTIES LINK_FLAGS "/PROFILE")
target_include_directories(
	Demo
	PRIVATE "glfw/include/"
	PRIVATE "glad/include/"
	PRIVATE "imgui"
	PUBLIC "include"
	PUBLIC "glm"
)

# TODO: Add tests and install targets if needed.

