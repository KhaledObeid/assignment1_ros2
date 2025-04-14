#ifndef BUMP_GO_NODE2B_HPP_
#define BUMP_GO_NODE2B_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace task2b
{

// FSM states.
enum State {
  FORWARD = 0,
  BACK = 1,
  TURN = 2,
  STOP = 3
};

class BumpGoNode2b : public rclcpp::Node
{
public:
  BumpGoNode2b();

private:
  // Callback and control methods.
  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
  void control_cycle();
  void go_state(int new_state);

  // Transition checkers.
  bool check_forward_2_back();
  bool check_forward_2_stop();
  bool check_stop_2_forward();
  bool check_back_2_turn();
  bool check_turn_2_forward();  // Closed-loop: returns true when the front region is clear.

  // Helper: returns the average clearance in a specified angular region.
  // New region definitions: for example, left (-45° to 0°), front (-5° to +5°), right (0° to 45°).
  double get_region_clearance(double angle_start, double angle_end);
  // Convenience: get average clearance around 0° (e.g. front region).
  double get_front_clearance();

  // ROS interfaces.
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Latest laser scan.
  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;

  // FSM state and timing.
  int state_;
  rclcpp::Time state_ts_;

  // Closed-loop turning: chosen turn direction.
  int turn_direction_;

  // Constants.
  static constexpr double SPEED_LINEAR       = 0.2;
  static constexpr double TURN_ANGULAR       = 0.5;    // Fixed turning speed [rad/s]
  // OBSTACLE_DISTANCE defines when the robot starts avoidance.
  static constexpr double OBSTACLE_DISTANCE  = 1.0;    
  // CLEAR_THRESHOLD: the front region (using a narrow window) must exceed this to exit TURN.
  static constexpr double CLEAR_THRESHOLD    = 3.0;    
  
  // Duration constants.
  static const rclcpp::Duration SCAN_TIMEOUT;
  static const rclcpp::Duration BACKING_TIME;
};

} // namespace task2b

#endif // BUMP_GO_NODE2B_HPP_
