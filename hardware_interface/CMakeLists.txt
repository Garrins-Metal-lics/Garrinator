cmake_minimum_required(VERSION 2.8.3)
project(garrinator_hardware_interface)

############
## Config ##
############

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	controller_manager
	hardware_interface
	combined_robot_hw
	joint_limits_interface
)

catkin_package(
	CATKIN_DEPENDS
		roscpp
		controller_manager
		hardware_interface
		combined_robot_hw
		joint_limits_interface)


include_directories(include
	${catkin_INCLUDE_DIRS} )


###########
## Build ##
###########

add_executable(hw_if_test
	src/test/hw_if_test.cpp
	src/odrive_hw_if.cpp)
add_dependencies(hw_if_test
	${catkin_EXPORTED_TARGETS} )
target_link_libraries(hw_if_test
	${catkin_LIBRARIES} )


add_executable(main_test
	src/test/main_test.cpp )
add_dependencies(main_test
	${catkin_EXPORTED_TARGETS} )
target_link_libraries(main_test
	${catkin_LIBRARIES} )
