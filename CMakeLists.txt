cmake_minimum_required(VERSION 3.0.2)
project(homographic_mono_cam_depth_perception)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  foxglove_msgs
  nav_msgs
  message_generation
)

find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ConeData.msg
  ConeBevData.msg
  FrameData.msg
  FrameBevData.msg
  HMatrix.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs cv_bridge foxglove_msgs nav_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Cone Base Detector
add_executable(cone_base_detector_node "src/detection/cone_base_detector.cpp")
add_dependencies(cone_base_detector_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(cone_base_detector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

## Cone Ground Projector
add_executable(cone_ground_projector_node "src/projection/cone_ground_projector.cpp")
add_dependencies(cone_ground_projector_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(cone_ground_projector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

## Standalone Cone Visualizer
add_executable(standalone_cone_visualizer_node "src/visualization/standalone_cone_visualizer.cpp")
add_dependencies(standalone_cone_visualizer_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(standalone_cone_visualizer_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

## Track Map Visualizer
add_executable(track_map_visualizer_node "src/visualization/track_map_visualizer.cpp")
add_dependencies(track_map_visualizer_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(track_map_visualizer_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

## BEV Cone Visualizer
add_executable(bev_cone_visualizer_node "src/visualization/bev_cone_visualizer.cpp")
add_dependencies(bev_cone_visualizer_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(bev_cone_visualizer_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

## H Matrix Publisher
add_executable(h_matrix_publisher_node "src/projection/h_matrix_publisher.cpp")
add_dependencies(h_matrix_publisher_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(h_matrix_publisher_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)


