cmake_minimum_required(VERSION 3.5)
project(roborts_base)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-w)
add_subdirectory(roborts_sdk)
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(roborts_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Glog REQUIRED)
# Include subdirectory

# Declare a ROS 2 executable
add_executable(roborts_base_node
  src/roborts_base_node.cpp
  src/chassis.cpp
  src/gimbal.cpp
  src/referee_system.cpp
)
target_link_libraries(roborts_base_node
        roborts_sdk)
# Specify libraries to link a library or executable target against
ament_target_dependencies(roborts_base_node
  rclcpp
  tf2_ros
  tf2
  roborts_msgs
  Threads
  nav_msgs
  tf2_geometry_msgs
  geometry_msgs
)

# Include directories
target_include_directories(roborts_base_node PUBLIC
  include
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${roborts_msgs_INCLUDE_DIRS}
  roborts_sdk/include
)

# ROS 2 exports and installations
ament_export_dependencies(rclcpp tf2_ros tf2 roborts_msgs)
ament_export_include_directories(include)
ament_export_libraries(
  roborts_sdk
)
install(TARGETS
  roborts_base_node
  DESTINATION lib/${PROJECT_NAME}
)

# Uncomment if there are specific messages or services
add_dependencies(roborts_base_node
  roborts_sdk)
ament_package()
