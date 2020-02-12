cmake_minimum_required(VERSION 2.8.3)
project(garrinator_hardware_interface)

############
## Config ##
############

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp)

catkin_package(
	CATKIN_DEPENDS
		roscpp)

###########
## Build ##
###########

include_directories(include
	${catkin_INCLUDE_DIRS} )
add_executable(main_test
	src/test/main_test.cpp )
add_dependencies(main_test
	${catkin_EXPORTED_TARGETS} )
target_link_libraries(main_test
	${catkin_LIBRARIES} )
