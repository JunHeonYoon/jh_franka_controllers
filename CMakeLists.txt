cmake_minimum_required(VERSION 3.10)
project(jh_franka_controllers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}" )

add_definitions(-DBUILD_DIRECTORY=\"${CMAKE_SOURCE_DIR}\") 
message(STATUS "BUILD_DIRECTORY = ${CMAKE_SOURCE_DIR}")

find_package(catkin REQUIRED COMPONENTS
roscpp
dynamic_reconfigure
franka_hw
franka_gripper
geometry_msgs
nav_msgs
hardware_interface
pluginlib
realtime_tools
roscpp
rospy
roslib
tf2
tf2_ros
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.5.0 REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)

if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP found, enabling support")
    add_definitions(-DUSE_OPENMP)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    franka_hw
    nav_msgs
    geometry_msgs
    hardware_interface
    pluginlib
    realtime_tools
    roscpp
    tf2 
    tf2_ros
  DEPENDS Franka
)

###########
## Build ##
###########

include_directories(
  include
  ${RBDL_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
src/default_controller.cpp
)

add_dependencies(jh_franka_controllers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
  ${RBDL_LIBRARY}
  Threads::Threads
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
endif()
#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES ${PROJECT_NAME}_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)