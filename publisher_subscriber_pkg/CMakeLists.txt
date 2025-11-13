cmake_minimum_required(VERSION 3.8)
project(publisher_subscriber_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_robot_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_executable(talker src/publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(turtle_mover src/turtlesim_pub.cpp)
ament_target_dependencies(turtle_mover rclcpp geometry_msgs)

add_executable(my_sensor src/custom_msg.cpp)
ament_target_dependencies(my_sensor rclcpp my_robot_msgs)

add_executable(service_server src/add_two_ints_server.cpp)
ament_target_dependencies(service_server rclcpp my_robot_msgs)

add_executable(service_client src/add_two_ints_client.cpp)
ament_target_dependencies(service_client rclcpp my_robot_msgs)

add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server rclcpp my_robot_msgs rclcpp_action)

add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client rclcpp my_robot_msgs rclcpp_action)

add_executable(simple_tf_kinematics src/simple_tf_kinematics.cpp)
ament_target_dependencies(simple_tf_kinematics rclcpp geometry_msgs tf2 tf2_ros)


install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  talker
  listener
  turtle_mover
  my_sensor
  service_server
  service_client
  action_server
  action_client
  simple_tf_kinematics
  DESTINATION lib/${PROJECT_NAME})


  install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


  
ament_package()
