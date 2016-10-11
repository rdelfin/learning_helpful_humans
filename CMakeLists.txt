cmake_minimum_required(VERSION 2.8.3)
project(learning_helpful_humans)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

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
  pcl_conversions
  pcl_ros
)

catkin_package(CATKIN_DEPENDS actionlib_msgs bwi_msgs std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(record_image_data src/record_image_data.cpp)
add_dependencies(record_image_data ${catkin_EXPORTED_TARGETS})
target_link_libraries(record_image_data ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(record_data_agent src/record_data_agent.cpp)
add_dependencies(record_data_agent ${catkin_EXPORTED_TARGETS})
target_link_libraries(record_data_agent ${catkin_LIBRARIES})

add_executable(ask_location_node src/ask_location_node.cpp)
add_dependencies(ask_location_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ask_location_node ${catkin_LIBRARIES})


add_executable(ask_location_node_tester src/ask_location_node_tester.cpp)
add_dependencies(ask_location_node_tester ${catkin_EXPORTED_TARGETS})
target_link_libraries(ask_location_node_tester ${catkin_LIBRARIES})

add_executable(ask_people_node src/ask_people_node.cpp
                               src/locations/AskLocation.cpp
                               src/locations/CorridorLocation.cpp
                               src/locations/LabLocation.cpp
                               src/locations/OfficeLocation.cpp)
add_dependencies(ask_people_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ask_people_node ${catkin_LIBRARIES})
