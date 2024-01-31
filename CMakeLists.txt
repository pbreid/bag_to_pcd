cmake_minimum_required(VERSION 3.0.2)
project(bag_to_pcd)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_conversions
  pcl_ros
  tf2_ros
  tf
  tf2
  tf2_msgs
  tf2_sensor_msgs
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs  
)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
##add_message_files(
##  FILES
##  voxelGrid.msg
##)
##generate_messages(
 ## DEPENDENCIES
  ##std_msgs
##)
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES bag_to_pcd 
  CATKIN_DEPENDS roscpp
                   pcl_conversions
                   pcl_ros
                   tf2
                   tf                   
                   tf2_ros
                   tf2_msgs
                    tf2_sensor_msgs
                    tf2_geometry_msgs
                    geometry_msgs
                    sensor_msgs
  DEPENDS EIGEN3
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)



## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(bag_to_pcd src/bag_to_pcd_node.cpp)
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(bag_to_pcd  ${catkin_LIBRARIES}  ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})


install(TARGETS bag_to_pcd
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)