cmake_minimum_required(VERSION 2.8.3)
project(save_rgbd_from_kinect2)
add_compile_options(-std=c++11)
find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(PCL REQUIRED COMPONENTS common io visualization)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# set(BOOST_ROOT "/usr/lib/x86_64-linux-gnu")
set(Boost_LIBRARIES "/usr/lib/x86_64-linux-gnu")
if(Boost_FOUND)
  MESSAGE( STATUS "Boost_INCLUDE_DIRS = ${Boost_INCLUDE_DIRS}.")
  MESSAGE( STATUS "Boost_LIBRARIES = ${Boost_LIBRARIES}.")
  MESSAGE( STATUS "Boost_LIB_VERSION = ${Boost_LIB_VERSION}.")
else()
  MESSAGE( WARNING "Not found Boost")
endif()

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	rostime sensor_msgs
	message_filters
	cv_bridge
	image_transport
	compressed_image_transport
	compressed_depth_image_transport
	visualization_msgs
	cmake_modules
	tf
	tf_conversions
	eigen_conversions
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES save_rgbd_from_kinect2
#  CATKIN_DEPENDS roscpp rospy stdmsg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${kinect2_bridge_INCLUDE_DIRS}
)


add_executable(save_depth src/save_depth.cpp)
target_link_libraries(save_depth
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${kinect2_bridge_LIBRARIES}
)
add_executable(save_rgb src/save_rgb.cpp)
target_link_libraries(save_rgb
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${kinect2_bridge_LIBRARIES}
)