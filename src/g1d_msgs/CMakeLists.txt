cmake_minimum_required(VERSION 3.16)
project(g1d_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GuardState.msg"
  "msg/BodyLock.msg"
  "msg/LioHealth.msg"
  "srv/SaveMap.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
