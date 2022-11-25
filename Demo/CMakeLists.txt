# CMakeList.txt : CMake project for LittlePhysics, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# Add source to this project's executable.
add_executable (Demo main.cpp)

target_link_libraries(
	Demo 
	LittlePhysics
)