#include <memory>
#include "BumpGoNode1.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  // Initialize ROS 2.
  rclcpp::init(argc, argv);

  // Create the node instance.
  auto node = std::make_shared<task1::BumpGoNode1>();
  rclcpp::spin(node);

  // Shutdown.
  rclcpp::shutdown();
  return 0;
}
