cmake_minimum_required(VERSION 2.8.3)
project(student_project)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
find_package(project_interface REQUIRED)

## Specify additional locations of header files
include_directories(
	include
	${OpenCV_INCLUDE_DIRS}
	${project_interface_INCLUDE_DIRS}
	src/boost/boost
)


## CREATE EXECUTABLE LIBRARY
add_library(student SHARED
	src/student_interface.cpp
	src/offset.cpp
	include/clipper/clipper.cpp
	src/intersection.cpp
	src/vertical_cell_decomposition.cpp
	src/dubins.cpp
	src/planning.cpp
)

target_link_libraries(
	student
	${OpenCV_LIBRARIES}
)

