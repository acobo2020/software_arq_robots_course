cmake_minimum_required(VERSION 2.8.3)
project(cameras_cpp)

set (CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  sensor_msgs
  std_msgs
  tf
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  darknet_ros_msgs
  message_generation
)

find_package(OpenCV)

add_message_files(
  FILES 
  bbx_custom.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
CATKIN_DEPENDS   
  roscpp
  sensor_msgs
  std_msgs
  tf
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
	${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(nodo_camera src/nodo_camera.cpp)
target_link_libraries(nodo_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(nodo_rgbd_filter src/nodo_rgbd_filter.cpp)
target_link_libraries(nodo_rgbd_filter ${catkin_LIBRARIES})

add_executable(nodo_rgbd_tf src/nodo_rgbd_tf.cpp)
target_link_libraries(nodo_rgbd_tf ${catkin_LIBRARIES})

add_executable(nodo_bbx src/nodo_bbx.cpp)
target_link_libraries(nodo_bbx ${catkin_LIBRARIES})
add_dependencies(nodo_bbx cameras_cpp_generate_messages_cpp)

set(ROSLINT_CPP_OPTS "--filter=-runtime/printf")


roslint_cpp(
  src/nodo_camera.cpp
  src/nodo_rgbd_filter.cpp
  src/nodo_rgbd_tf.cpp
)
