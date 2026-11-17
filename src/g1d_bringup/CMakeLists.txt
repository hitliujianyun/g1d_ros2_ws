cmake_minimum_required(VERSION 3.16)
project(g1d_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/g1d_env.sh DESTINATION share/${PROJECT_NAME}/scripts)

ament_package()
