cmake_minimum_required(VERSION 2.8.3)
project(onboard_px4)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  aerialcore_common
  geodesy
  mavros
  mavros_msgs
  mission_lib
  roscpp
  std_msgs
  std_srvs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  # INCLUDE_DIRS
  # LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS aerialcore_common geodesy mavros mavros_msgs mission_lib roscpp std_msgs std_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(px4_mission_node src/px4_mission_node.cpp)
add_dependencies(px4_mission_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(px4_mission_node ${catkin_LIBRARIES})
