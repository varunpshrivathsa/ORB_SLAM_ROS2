cmake_minimum_required(VERSION 3.8)
project(orbslam_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(Pangolin REQUIRED)

# ---- ORB-SLAM3 paths ----
set(ORB_SLAM3_DIR /workspace/ORB_SLAM3)

set(ORB_SLAM3_INCLUDE_DIRS
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
  ${ORB_SLAM3_DIR}/Thirdparty/g2o
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/DBoW2
)

include_directories(
  include
  ${ORB_SLAM3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/include/eigen3
  ${Pangolin_INCLUDE_DIRS}
)

link_directories(
  ${ORB_SLAM3_DIR}/lib
)

add_executable(rgbd_node src/rgbd_node.cpp)

ament_target_dependencies(rgbd_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  cv_bridge
  image_transport
  message_filters
)

target_link_libraries(rgbd_node
  ${OpenCV_LIBS}
  ORB_SLAM3
  ${Pangolin_LIBRARIES}
  GLEW::GLEW
  OpenGL::GL
)

install(TARGETS
  rgbd_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()