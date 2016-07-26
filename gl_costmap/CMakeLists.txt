cmake_minimum_required(VERSION 2.8.3)
project(gl_costmap)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  tf
  octomap_ros
  octomap_msgs
)


find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(octomap REQUIRED)


catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(${OCTOMAP_INCLUDE_DIRS})



add_executable(data_pretreatment src/data_pretreatment.cpp)

add_dependencies(data_pretreatment ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(data_pretreatment ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
link_libraries(${OCTOMAP_LIBRARIES})