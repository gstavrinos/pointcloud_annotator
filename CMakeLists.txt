cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_annotator)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  pcl_conversions
  message_generation
)

add_service_files(DIRECTORY srv
    FILES
    Update.srv
)

generate_messages(DEPENDENCIES sensor_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud_annotator src/pointcloud_annotator.cpp)
target_link_libraries(pointcloud_annotator ${catkin_LIBRARIES})

add_executable(annotated_pointcloud_publisher src/publisher.cpp)
target_link_libraries(annotated_pointcloud_publisher ${catkin_LIBRARIES})
add_dependencies(annotated_pointcloud_publisher ${pointcloud_annotator_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})