cmake_minimum_required(VERSION 2.8.3)
project(omni_4_controller)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib
  roscpp
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(my_controller_lib src/omni_4_controller.cpp)

target_link_libraries(my_controller_lib ${catkin_LIBRARIES})
