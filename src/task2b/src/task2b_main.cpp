#include <memory>
#include "BumpGoNode2b.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<task2b::BumpGoNode2b>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
