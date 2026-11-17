cmake_minimum_required(VERSION 3.16)
project(g1d_description NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config launch meshes DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/fetch_meshes.py DESTINATION lib/${PROJECT_NAME})

ament_package()
