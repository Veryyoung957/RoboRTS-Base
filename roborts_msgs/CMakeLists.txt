cmake_minimum_required(VERSION 3.5)
project(roborts_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/chassis/TwistAccel.msg"
  "msg/gimbal/GimbalAngle.msg"
  "msg/referee_system/GameStatus.msg"
  "msg/referee_system/GameResult.msg"
  "msg/referee_system/GameRobotHP.msg"
  "msg/referee_system/GameRobotBullet.msg"
  "msg/referee_system/GameZone.msg"
  "msg/referee_system/GameZoneArray.msg"
  "msg/referee_system/RobotStatus.msg"
  "msg/referee_system/RobotHeat.msg"
  "msg/referee_system/RobotDamage.msg"
  "msg/referee_system/RobotShoot.msg"
  "msg/referee_system/LurkStatus.msg"
  "srv/gimbal/FricWhl.srv"
  "srv/gimbal/ShootCmd.srv"
)


ament_package()
