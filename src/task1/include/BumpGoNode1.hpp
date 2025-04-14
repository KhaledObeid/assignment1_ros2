#ifndef BUMP_GO_NODE1_HPP_
#define BUMP_GO_NODE1_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace task1
{

// Define state values.
enum State {
  FORWARD = 0,
  BACK = 1,
  TURN = 2,
  STOP = 3
};

class BumpGoNode1 : public rclcpp::Node
{
public:
  BumpGoNode1();

private:
  // Callback and control cycle functions.
  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
  void control_cycle();
  void go_state(int new_state);

  // State transition checkers.
  bool check_forward_2_back();
  bool check_forward_2_stop();
  bool check_stop_2_forward();
  bool check_back_2_turn();
  bool check_turn_2_forward();

  // NEW: Helper to determine turn direction.
  // Returns +1 for left, -1 for right.
  int determine_turn_direction();

  // ROS interfaces.
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Store the last laser scan.
  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;

  // State and timing.
  int state_;
  rclcpp::Time state_ts_;

  // NEW: Turn direction selected based on sensor data.
  // +1 means turn left; -1 means turn right.
  int turn_direction_;

  // Constants.
  static constexpr double SPEED_LINEAR = 0.2;
  static constexpr double SPEED_ANGULAR = 0.5;
  static constexpr double OBSTACLE_DISTANCE = 0.5;

  // rclcpp::Duration is non-literal; declare as static const.
  static const rclcpp::Duration SCAN_TIMEOUT;
  static const rclcpp::Duration BACKING_TIME;
  static const rclcpp::Duration TURNING_TIME;
};

}  // namespace task1

#endif  // BUMP_GO_NODE1_HPP_
