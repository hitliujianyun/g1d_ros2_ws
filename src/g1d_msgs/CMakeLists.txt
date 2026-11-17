cmake_minimum_required(VERSION 3.16)
project(g1d_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GuardState.msg"
  "msg/BodyLock.msg"
  "msg/LioHealth.msg"
  "msg/LocalizationStatus.msg"
  "srv/SaveMap.srv"
  "srv/Relocalize.srv"
  "action/NavigateRelative.action"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
