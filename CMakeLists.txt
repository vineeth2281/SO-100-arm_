cmake_minimum_required(VERSION 3.8)
project(so_100_arm)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)


install(
  DIRECTORY 
    config
    launch
    models
	  urdf
  DESTINATION 
    share/${PROJECT_NAME}
)

ament_package()
