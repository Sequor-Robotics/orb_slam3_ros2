cmake_minimum_required(VERSION 3.5)
project(orbslam3)

# You should set the PYTHONPATH to your own python site-packages path
set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages")

# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
set(CMAKE_MODULE_PATH 
${CMAKE_MODULE_PATH} 
${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules) 

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(epoxy REQUIRED)
find_package(OpenGL REQUIRED COMPONENTS OpenGL EGL)
find_package(ORB_SLAM3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV 4.0 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.2 not found.")
   endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

# Add the directory containing libORB_SLAM3.so to the linker search path
link_directories(~/thirdparty/ORB-SLAM3-STEREO-FIXED/Thirdparty/DBoW2/lib)
link_directories(~/thirdparty/ORB-SLAM3-STEREO-FIXED/Thirdparty/g2o/lib)
link_directories(~/thirdparty/ORB-SLAM3-STEREO-FIXED/lib)

add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)

ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin visualization_msgs)
target_include_directories(
  mono 
  PUBLIC 
  ${PROJECT_SOURCE_DIR}/include ${ORB_SLAM3_INCLUDE_DIRS}/include ${ORB_SLAM3_INCLUDE_DIRS}/include/CameraModels
)

add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)
ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin visualization_msgs)
target_include_directories(
  rgbd
  PUBLIC 
  ${PROJECT_SOURCE_DIR}/include ${ORB_SLAM3_INCLUDE_DIRS}/include ${ORB_SLAM3_INCLUDE_DIRS}/include/CameraModels
)

add_executable(stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
  src/ros_utils.cpp
)
ament_target_dependencies(stereo rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros cv_bridge message_filters ORB_SLAM3 Pangolin visualization_msgs)
target_include_directories(
  stereo
  PUBLIC 
  ${PROJECT_SOURCE_DIR}/include ${ORB_SLAM3_INCLUDE_DIRS}/include ${ORB_SLAM3_INCLUDE_DIRS}/include/CameraModels
)

target_link_libraries(mono ${OpenCV_LIBS} ${ORB_SLAM3_LIBRARIES})
target_link_libraries(rgbd ${OpenCV_LIBS} ${ORB_SLAM3_LIBRARIES})
target_link_libraries(stereo ${OpenCV_LIBS} ${ORB_SLAM3_LIBRARIES})

# Streo-Inertial SLAM
add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
  src/ros_utils.cpp
)
ament_target_dependencies(stereo-inertial rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros cv_bridge ORB_SLAM3 Pangolin visualization_msgs)
target_include_directories(
  stereo-inertial
  PUBLIC 
  ${PROJECT_SOURCE_DIR}/include ${ORB_SLAM3_INCLUDE_DIRS}/include ${ORB_SLAM3_INCLUDE_DIRS}/include/CameraModels
)
target_link_libraries(stereo-inertial ${OpenCV_LIBS} ${ORB_SLAM3_LIBRARIES})

install(TARGETS mono rgbd stereo stereo-inertial
  DESTINATION lib/${PROJECT_NAME})

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install vocab files
install(DIRECTORY
  vocabulary
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
