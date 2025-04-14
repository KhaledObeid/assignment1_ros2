#include <utility>
#include <cmath> // for M_PI, fabs
#include "t2a/BumpGoNode.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

namespace t2a
{

using namespace std::chrono_literals;
using std::placeholders::_1;

BumpGoNode::BumpGoNode()
: Node("bump_go"),
  state_(FORWARD)
{
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS(),
    std::bind(&BumpGoNode::scan_callback, this, _1));

  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
  timer_ = create_wall_timer(50ms, std::bind(&BumpGoNode::control_cycle, this));

  state_ts_ = now();
}

void
BumpGoNode::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  last_scan_ = std::move(msg);
}

void
BumpGoNode::control_cycle()
{
  // Do nothing until the first sensor read
  if (last_scan_ == nullptr) { return; }

  geometry_msgs::msg::Twist out_vel;

  switch (state_) {
    case FORWARD:
      out_vel.linear.x = SPEED_LINEAR;
      if (check_forward_2_stop()) {
        go_state(STOP);
      }
      if (check_forward_2_back()) {
        go_state(BACK);
      }
      break;

    case BACK:
      out_vel.linear.x = -SPEED_LINEAR;
      if (check_back_2_turn()) {
        go_state(TURN);
      }
      break;

    case TURN:
    {
      // --- NEW CODE START: Turn exactly to the target angle ---
      // We scan the left and right sectors to find the maximum free distance and record its angle.
      // Then compute the duration needed to rotate by that angle at SPEED_ANGULAR.
      // Log the computed target angle and turn time for debugging.

      static bool turn_configured = false;
      static rclcpp::Time turn_start_time;
      static rclcpp::Duration target_turn_duration(0, 0);  // Initialize with 0 seconds, 0 nanoseconds.
      static double target_angle = 0.0; // desired relative turn (radians)

      if (!turn_configured) {
        double left_max = 0.0;
        double right_max = 0.0;
        double left_best_angle = 0.0;
        double right_best_angle = 0.0;

        // Iterate over the scan readings to determine the best angle in each sector.
        double angle = last_scan_->angle_min;
        for (size_t i = 0; i < last_scan_->ranges.size(); i++) {
          // Left sector: 45° to 90° (π/4 to π/2 radians)
          if (angle >= M_PI/4 && angle <= M_PI/2) {
            if (last_scan_->ranges[i] > left_max) {
              left_max = last_scan_->ranges[i];
              left_best_angle = angle;
            }
          }
          // Right sector: -90° to -45° (-π/2 to -π/4 radians)
          if (angle >= -M_PI/2 && angle <= -M_PI/4) {
            if (last_scan_->ranges[i] > right_max) {
              right_max = last_scan_->ranges[i];
              right_best_angle = angle;
            }
          }
          angle += last_scan_->angle_increment;
        }

        // Choose the sector with the greater maximum free space.
        // If both values are equal, we default to turning right.
        if (left_max > right_max) {
          target_angle = left_best_angle; // positive value: turn left
        } else {
          target_angle = right_best_angle; // negative value: turn right
        }

        // Compute the turn duration: time = |angle| / angular speed.
        double turn_time_sec = std::fabs(target_angle) / SPEED_ANGULAR;
        target_turn_duration = rclcpp::Duration::from_seconds(turn_time_sec);

        // Log the computed target angle and turn duration.
        RCLCPP_INFO(get_logger(), "Target angle: %.2f rad, Turn duration: %.2f sec", target_angle, turn_time_sec);

        // Record the start time of the turn and mark turn as configured.
        turn_start_time = now();
        turn_configured = true;
      }

      // Command the robot to turn at the constant angular speed.
      if (target_angle >= 0) {
        out_vel.angular.z = SPEED_ANGULAR;  // Turn left.
      } else {
        out_vel.angular.z = -SPEED_ANGULAR;   // Turn right.
      }

      // When the elapsed time exceeds the computed duration, finish turning.
      if ((now() - turn_start_time) >= target_turn_duration) {
        turn_configured = false;  // Reset turning configuration.
        go_state(FORWARD);
      }
      // --- NEW CODE END ---
      break;
    }

    case STOP:
      if (check_stop_2_forward()) {
        go_state(FORWARD);
      }
      break;
  }

  vel_pub_->publish(out_vel);
}

void
BumpGoNode::go_state(int new_state)
{
  state_ = new_state;
  state_ts_ = now();
}

bool
BumpGoNode::check_forward_2_back()
{
  // Detect an obstacle in front.
  size_t center = last_scan_->ranges.size() / 2;
  return last_scan_->ranges[center] < OBSTACLE_DISTANCE;
}

bool
BumpGoNode::check_forward_2_stop()
{
  // Stop if no sensor readings for 1 second.
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed > SCAN_TIMEOUT;
}

bool
BumpGoNode::check_stop_2_forward()
{
  // Resume forward if sensor readings are available again.
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed < SCAN_TIMEOUT;
}

bool
BumpGoNode::check_back_2_turn()
{
  // Stay in BACK state for a set amount of time (BACKING_TIME).
  return (now() - state_ts_) > BACKING_TIME;
}

bool
BumpGoNode::check_turn_2_forward()
{
  // Not used in the new TURN implementation.
  return false;
}

}  // namespace t2a
