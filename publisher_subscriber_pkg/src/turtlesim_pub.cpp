#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"

// #include "

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("turtlesim_pub"), count_(0)
    {
      this->declare_parameter("robot_speed", 0.1);
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
      timer_ = this->create_wall_timer(
      1s, std::bind(&MinimalPublisher::timer_callback, this));

    }

  private:
    void timer_callback()
    {
      auto message = geometry_msgs::msg::Twist();
    //   message.data = "Hello, world! " + std::to_string(count_++);
    //   RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

    double my_param = this->get_parameter("robot_speed").as_double();

    message.linear.x = my_param;
    message.angular.z = my_param;

    // message.linear.y = 0/0;



    RCLCPP_INFO(this->get_logger(), "Publishing: robot x linear speed = '%f'", message.linear.x);
    RCLCPP_INFO(this->get_logger(), "Publishing: robot z angular speed = '%f'", message.angular.z);

    // RCLCPP_WARN(this->get_logger(), "Publishing: robot z angular speed = '%f'", message.angular.z);
    // RCLCPP_DEBUG(this->get_logger(), "Publishing: robot z angular speed = '%f'", message.angular.z);
    // RCLCPP_ERROR(this->get_logger(), "Publishing: robot z angular speed = '%f'", message.angular.z);


      publisher_->publish(message);
      // this->set_parameters(robot_speed);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    size_t count_;
    std::vector<rclcpp::Parameter> robot_speed{rclcpp::Parameter("robot_speed", 0.1)};

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}