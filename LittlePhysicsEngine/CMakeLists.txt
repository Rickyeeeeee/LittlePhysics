# CMakeList.txt : CMake project for LittlePhysics, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# Add source to this project's executable.
add_library (LittlePhysics STATIC "LittlePhysics.cpp" "LittlePhysics.h")

# TODO: Add tests and install targets if needed.
