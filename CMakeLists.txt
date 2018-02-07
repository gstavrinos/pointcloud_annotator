cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_annotator)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud_annotator src/pointcloud_annotator.cpp)
target_link_libraries(pointcloud_annotator ${catkin_LIBRARIES})