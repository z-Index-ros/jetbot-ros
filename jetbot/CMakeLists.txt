cmake_minimum_required(VERSION 3.5)
project(jetbot)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros2_numpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

##############
ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_package()
