#include "rclcpp/rclcpp.hpp"
#include "my_robot_msgs/srv/sum.hpp"

#include <memory>

void add(const std::shared_ptr<my_robot_msgs::srv::Sum::Request> request,
          std::shared_ptr<my_robot_msgs::srv::Sum::Response> response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

  rclcpp::Service<my_robot_msgs::srv::Sum>::SharedPtr service = node->create_service<my_robot_msgs::srv::Sum>("add_two_ints", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}