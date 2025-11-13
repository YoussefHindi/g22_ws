#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "my_robot_msgs/msg/my_new_sensor.hpp"                                            // CHANGE

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<my_robot_msgs::msg::MyNewSensor>("my_sensor_topic", 10);  // CHANGE
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = my_robot_msgs::msg::MyNewSensor();                                   
    message.sensor_id = 1;                                                     
    message.temperature = 30.0; 
    message.air_pressure = 1.1; 
    message.air_speed = 1.4; 
    message.center.x = 2;
    message.center.y = 2;
    message.center.z = 5;

    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.sensor_id << "'");    
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.temperature << "'");    
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.air_pressure << "'");    
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.air_speed << "'");    
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.center.x << "'");    
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.center.y << "'");  
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.center.z << "'");    
  


    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<my_robot_msgs::msg::MyNewSensor>::SharedPtr publisher_;             // CHANGE
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}