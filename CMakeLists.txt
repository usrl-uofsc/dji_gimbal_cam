cmake_minimum_required(VERSION 2.8.3)
project(dji_camera)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  dji_sdk
  geometry_msgs
  message_generation
  std_srvs

)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dji_camera dji_gimbal
  CATKIN_DEPENDS camera_info_manager cv_bridge image_transport roscpp sensor_msgs std_msgs dji_sdk geometry_msgs roscpp message_runtime
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(/usr/lib /usr/local/lib usr/lib/arm-linux-gnueabihf libs)

add_executable(dji_camera_node
  src/dji_camera.cpp
)
add_executable(dji_gimbal_node
	src/dji_gimbal.cpp
)

add_dependencies(dji_gimbal_node ${catkin_EXPORTED_TARGETS} )

target_link_libraries(dji_camera_node
  dcam
  ${DJIOSDK_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_link_libraries(dji_gimbal_node
  ${DJIOSDK_LIBRARIES}
  ${catkin_LIBRARIES}
)
