#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/header.hpp>

class MapPublisherNode : public rclcpp::Node
{
public:
  MapPublisherNode()
  : Node("map_publisher_node")
  {
    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    timer_ = this->create_wall_timer(
      std::chrono::microseconds(10),
      std::bind(&MapPublisherNode::publish_map, this));
  }

private:
  void publish_map()
  {
    auto map_msg = nav_msgs::msg::OccupancyGrid();

    // Set map metadata
    map_msg.header.stamp = this->now();
    map_msg.header.frame_id = "map";
    map_msg.info.resolution = 0.1; // or 10 cm per cell
    // map_msg.info.width = 10;
    // map_msg.info.height = 10;
    map_msg.info.width = 100;
    map_msg.info.height = 100;
    // map_msg.info.origin.position.x = -5.0;
    // map_msg.info.origin.position.y = -5.0;

    map_msg.info.origin.position.x = - (100 * 0.1) / 2.0; 
    map_msg.info.origin.position.y = - (100 * 0.1) / 2.0; 

    map_msg.info.origin.position.z = 0.0;
    map_msg.info.origin.orientation.w = 1.0; // no rotation

    // Fill data with occupancy values
    // map_msg.data.resize(100, 0); // 10x10 cells initialized to 0 (free)
    map_msg.data.resize(100 * 100, 0); // 100x100 cells

    for (int y = 0; y < 10; ++y) {
      for (int x = 0; x < 10; ++x) {
        if (x == 0 || x == 9 || y == 0 || y == 9) {
          map_msg.data[y * 10 + x] = 100; // border occupied
        }
      }
    }

    for (int y = 0; y < 100; ++y) {
        for (int x = 0; x < 100; ++x) {
          if (x == 0 || x == 99 || y == 0 || y == 99) {
          map_msg.data[y * 100 + x] = 100; // Occupied border
          }
        }
    }

    publisher_->publish(map_msg);
    RCLCPP_INFO(this->get_logger(), "Published occupancy grid map");
  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
