#include <utility>
#include <cmath> // for M_PI
#include "t1/BumpGoNode.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

namespace t1
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
      // Compute average distances for left and right segments.
      // Left segment: angles between 45° and 90° (i.e., π/4 to π/2 radians).
      // Right segment: angles between -90° and -45° (i.e., -π/2 to -π/4 radians).
      double left_sum = 0.0;
      int left_count = 0;
      double right_sum = 0.0;
      int right_count = 0;

      // Iterate over the scan readings.
      double angle = last_scan_->angle_min;
      for (size_t i = 0; i < last_scan_->ranges.size(); i++) {
        // For left side (45° to 90°)
        if (angle >= M_PI/4 && angle <= M_PI/2) {
          left_sum += last_scan_->ranges[i];
          left_count++;
        }
        // For right side (-90° to -45°)
        if (angle >= -M_PI/2 && angle <= -M_PI/4) {
          right_sum += last_scan_->ranges[i];
          right_count++;
        }
        angle += last_scan_->angle_increment;
      }

      double left_avg = (left_count > 0) ? (left_sum / left_count) : 0.0;
      double right_avg = (right_count > 0) ? (right_sum / right_count) : 0.0;
      
      // Log the detection values on screen.
      RCLCPP_INFO(get_logger(), "Left detection: %.2f, Right detection: %.2f", left_avg, right_avg);

      // Choose turning direction:
      // If left average is greater than right average, turn left.
      // Otherwise (if right is greater or if both are equal), turn right.
      if (left_avg > right_avg) {
        out_vel.angular.z = SPEED_ANGULAR;  // Turn left (positive angular velocity)
      } else {
        out_vel.angular.z = -SPEED_ANGULAR; // Turn right (negative angular velocity)
      }

      if (check_turn_2_forward()) {
        go_state(FORWARD);
      }
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
  // Stop if no sensor readings for 1 second
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed > SCAN_TIMEOUT;
}

bool
BumpGoNode::check_stop_2_forward()
{
  // Resume forward if sensor readings are available again
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed < SCAN_TIMEOUT;
}

bool
BumpGoNode::check_back_2_turn()
{
  // Stay in BACK state for a set amount of time (BACKING_TIME)
  return (now() - state_ts_) > BACKING_TIME;
}

bool
BumpGoNode::check_turn_2_forward()
{
  // Stay in TURN state for a set amount of time (TURNING_TIME)
  return (now() - state_ts_) > TURNING_TIME;
}

}  // namespace t1
