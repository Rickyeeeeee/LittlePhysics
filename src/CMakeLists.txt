# CMakeList.txt : CMake project for LittlePhysics, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# Add source to this project's executable.
add_library (LittlePhysics STATIC "LittlePhysics.cpp" "Collision/CollisionNarrowPhase.cpp" "World.cpp" "Body.cpp" "Shape.cpp" "Collision/CollisionBroadPhase.cpp" "Collision/CollisionManager.cpp")

target_include_directories(
	LittlePhysics
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
# TODO: Add tests and install targets if needed.
