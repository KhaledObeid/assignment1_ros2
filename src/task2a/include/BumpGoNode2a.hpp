#ifndef BUMP_GO_NODE2A_HPP_
#define BUMP_GO_NODE2A_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace task2a
{

// FSM states.
enum State {
  FORWARD = 0,
  BACK = 1,
  TURN = 2,
  STOP = 3
};

class BumpGoNode2a : public rclcpp::Node
{
public:
  BumpGoNode2a();

private:
  // Callback and control functions.
  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
  void control_cycle();
  void go_state(int new_state);

  // Transition checkers.
  bool check_forward_2_back();
  bool check_forward_2_stop();
  bool check_stop_2_forward();
  bool check_back_2_turn();
  bool check_turn_2_forward();  // For open-loop, this function is not used to decide state.

  // Helper: compute target turn angle (in radians) from the laser scan.
  double compute_target_turn_angle();

  // ROS interfaces.
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Storage for latest laser scan.
  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;

  // FSM state and timing.
  int state_;
  rclcpp::Time state_ts_;

  // Parameters for open-loop turning.
  int turn_direction_;        // +1 indicates turning left; -1 turning right.
  double target_turn_angle_;  // Absolute angle (radians) to turn.
  double turn_duration_;      // Computed turn duration (in seconds).
  double computed_angular_speed_;  // Dynamically computed angular speed (rad/s).

  // Constants.
  static constexpr double SPEED_LINEAR = 0.2;
  static constexpr double OBSTACLE_DISTANCE = 0.5;
  
  // Remove fixed angular speed constant; instead, define minimum and maximum.
  static constexpr double MIN_ANGULAR_SPEED = 0.3;
  static constexpr double MAX_ANGULAR_SPEED = 1.0;

  // rclcpp::Duration is non‐literal, so declare as static const.
  static const rclcpp::Duration SCAN_TIMEOUT;
  static const rclcpp::Duration BACKING_TIME;
  // TURNING_TIME is used as a maximum safeguard (optional).
  static const rclcpp::Duration TURNING_TIME;
};

} // namespace task2a

#endif // BUMP_GO_NODE2A_HPP_
