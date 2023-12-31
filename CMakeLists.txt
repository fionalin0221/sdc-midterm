cmake_minimum_required(VERSION 3.0.2)
project(localization)

set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")
find_package(catkin REQUIRED COMPONENTS
  OpenCV
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  std_msgs
  tf
  tf_conversions
  cv_bridge
  image_transport
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  $(OpenCV_INCLUDE_DIRS)
)

add_executable(localization src/localization.cpp)
target_link_libraries(localization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(map_modified src/map_modified.cpp)
target_link_libraries(map_modified ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(map_pub src/map_pub.cpp)
target_link_libraries(map_pub ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(radar src/radar.cpp)
target_link_libraries(radar ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(visualization src/visualization.cpp)
target_link_libraries(visualization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(test1 src/test1.cpp)
target_link_libraries(test1 ${catkin_LIBRARIES})

add_executable(localization_copy src/localization_copy.cpp)
target_link_libraries(localization_copy ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(localization_copy_2 src/localization_copy_2.cpp)
target_link_libraries(localization_copy_2 ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(yanting src/yanting.cpp)
target_link_libraries(yanting ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})