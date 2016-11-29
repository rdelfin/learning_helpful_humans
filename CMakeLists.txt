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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O1")


# Boost library (used for UUID generation)
FIND_PACKAGE(Boost COMPONENTS system REQUIRED)

# we are now using the OpenCV library
find_package(OpenCV 2)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(CURL REQUIRED)

SET(FLTK_SKIP_FLUID TRUE)
FIND_PACKAGE(FLTK REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)
add_definitions(${FLTK_DEFINITIONS})

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

catkin_package(
        CATKIN_DEPENDS actionlib_msgs bwi_msgs std_msgs
        DEPENDS system_lib curl curlpp
        )

include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${FLTK_INCLUDE_DIR}
        ${CURL_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        )

add_executable(record_image_data src/record_image_data.cpp)
add_dependencies(record_image_data ${catkin_EXPORTED_TARGETS})
target_link_libraries(record_image_data ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(record_data_agent src/record_data_agent.cpp)
add_dependencies(record_data_agent ${catkin_EXPORTED_TARGETS})
target_link_libraries(record_data_agent ${catkin_LIBRARIES})

add_executable(image_asker_node src/image_asker_node.cpp
                                src/Fl_ViewerCV.cxx)
add_dependencies(image_asker_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_asker_node ${catkin_LIBRARIES})

target_link_libraries(image_asker_node ${OPENGL_LIBRARIES})
target_link_libraries(image_asker_node ${FLTK_LIBRARIES})



add_executable(image_asker_node_tester src/image_asker_node_tester.cpp)
add_dependencies(image_asker_node_tester ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_asker_node_tester ${catkin_LIBRARIES})

add_executable(location_selector_node src/location_selector_node.cpp
                                      src/locations/AskLocation.cpp
                                      src/locations/CorridorLocation.cpp
                                      src/locations/LabLocation.cpp
                                      src/locations/OfficeLocation.cpp)
add_dependencies(location_selector_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(location_selector_node ${catkin_LIBRARIES})

add_executable(location_asker_node src/location_asker_node.cpp)
add_dependencies(location_asker_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(location_asker_node ${catkin_LIBRARIES})


add_executable(image_tool_node src/image_tool_node.cpp
                               src/request/PostImageRequest.cpp
                               src/request/PostFieldValue.cpp
                               src/request/GetFieldValue.cpp
                               src/request/AppendFieldValue.cpp
                               src/request/GetImage.cpp
                               src/imagetool/DatabaseImage.cpp
                               src/imagetool/ImageMetadata.cpp
                               src/imagetool/ImagePickerPolicy.cpp
                               src/imagetool/RandomImagePolicy.cpp)
add_dependencies(image_tool_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_tool_node
        ${catkin_LIBRARIES}
        ${CURL_LIBRARIES}
        ${Boost_LIBRARIES}
        curlpp)