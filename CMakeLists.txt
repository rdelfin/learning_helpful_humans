cmake_minimum_required(VERSION 2.8.3)
project(learning_helpful_humans)

# we are now using the OpenCV library
find_package(OpenCV 2)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  bwi_kr_execution
  bwi_msgs
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  sensor_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS actionlib_msgs bwi_msgs std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(record_image_data src/record_image_data.cpp)
add_dependencies(record_image_data ${catkin_EXPORTED_TARGETS})
target_link_libraries(record_image_data ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

