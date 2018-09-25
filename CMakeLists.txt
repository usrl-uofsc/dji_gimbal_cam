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

link_directories(/usr/lib /usr/local/lib usr/lib/arm-linux-gnueabihf)

add_executable(dji_camera
	src/dji_camera.cpp
	)

target_link_libraries(dji_camera
	dcam
	${catkin_LIBRARIES}
	)