cmake_minimum_required(VERSION 3.0.2)
project(cartesian_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        hardware_interface
        roscpp
  )


catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES
  CATKIN_DEPENDS
    geometry_msgs
    hardware_interface
    roscpp
  #DEPENDS system_lib
  )

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(cartesian_state_interface_test test/cartesian_state_interface_test.cpp)
  target_link_libraries(cartesian_state_interface_test ${catkin_LIBRARIES})

  catkin_add_gtest(cartesian_command_interface_test test/cartesian_command_interface_test.cpp)
  target_link_libraries(cartesian_command_interface_test ${catkin_LIBRARIES})
endif()
