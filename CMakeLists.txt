# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.
#
cmake_minimum_required (VERSION 3.8)

project ("LittlePhysics")

# Include sub-projects.
add_subdirectory ("LittlePhysicsEngine")
add_subdirectory ("Demo")

target_include_directories(
	Demo
	PRIVATE "LittlePhysicsEngine"
)

# TODO: Add tests and install targets if needed.

