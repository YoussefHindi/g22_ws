cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# find_package(my_robot_controllers REQUIRED)


install(
  DIRECTORY meshes urdf launch rviz worlds models photos
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
