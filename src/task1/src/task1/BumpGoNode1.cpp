#include <utility>
#include <algorithm>
#include "BumpGoNode1.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace task1 {

using namespace std::chrono_literals;
using std::placeholders::_1;

// Define the static const Duration members.
const rclcpp::Duration BumpGoNode1::SCAN_TIMEOUT = rclcpp::Duration::from_seconds(0.5);
const rclcpp::Duration BumpGoNode1::BACKING_TIME  = rclcpp::Duration::from_seconds(1.0);
const rclcpp::Duration BumpGoNode1::TURNING_TIME  = rclcpp::Duration::from_seconds(2.0);

BumpGoNode1::BumpGoNode1()
: Node("bump_go"),
  state_(FORWARD),
  turn_direction_(+1)
{
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", 
    rclcpp::SensorDataQoS(),
    std::bind(&BumpGoNode1::scan_callback, this, _1)
  );

  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
  timer_ = create_wall_timer(50ms, std::bind(&BumpGoNode1::control_cycle, this));

  state_ts_ = now();
}

void BumpGoNode1::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  last_scan_ = std::move(msg);
}

void BumpGoNode1::control_cycle()
{
  if (last_scan_ == nullptr)
    return; // Wait for sensor data

  geometry_msgs::msg::Twist out_vel;

  switch (state_) {
    case FORWARD:
      out_vel.linear.x = SPEED_LINEAR;
      if (check_forward_2_stop()) {
        go_state(STOP);
      } else if (check_forward_2_back()) {
        // Decide which direction to turn based on diagonal readings.
        turn_direction_ = determine_turn_direction();
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
      out_vel.angular.z = turn_direction_ * SPEED_ANGULAR;
      if (check_turn_2_forward()) {
        go_state(FORWARD);
      }
      break;

    case STOP:
      if (check_stop_2_forward()) {
        go_state(FORWARD);
      }
      break;
  }

  vel_pub_->publish(out_vel);
}

void BumpGoNode1::go_state(int new_state)
{
  state_ = new_state;
  state_ts_ = now();
}

bool BumpGoNode1::check_forward_2_back()
{
  size_t center_index = last_scan_->ranges.size() / 2;
  return last_scan_->ranges[center_index] < OBSTACLE_DISTANCE;
}

bool BumpGoNode1::check_forward_2_stop()
{
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed > SCAN_TIMEOUT;
}

bool BumpGoNode1::check_stop_2_forward()
{
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed < SCAN_TIMEOUT;
}

bool BumpGoNode1::check_back_2_turn()
{
  return (now() - state_ts_) > BACKING_TIME;
}

bool BumpGoNode1::check_turn_2_forward()
{
  return (now() - state_ts_) > TURNING_TIME;
}

int BumpGoNode1::determine_turn_direction()
{
  size_t num_readings = last_scan_->ranges.size();
  size_t center_index = num_readings / 2;
  
  // Define an offset of 10% of the total readings.
  size_t offset = static_cast<size_t>(num_readings * 0.1);
  // Define the averaging window size.
  size_t window = 5;
  
  // Compute the average for the left side.
  size_t left_start = std::min(center_index + offset, num_readings - 1);
  if (left_start >= window / 2)
    left_start -= window / 2;
  size_t left_end = std::min(left_start + window - 1, num_readings - 1);
  
  double left_sum = 0.0;
  size_t left_count = 0;
  for (size_t i = left_start; i <= left_end; i++) {
    left_sum += last_scan_->ranges[i];
    ++left_count;
  }
  double left_avg = (left_count > 0) ? left_sum / left_count : 0.0;
  
  // Compute the average for the right side.
  size_t right_start = (center_index >= offset) ? center_index - offset : 0;
  //if (right_start >= window / 2)
    right_start -= window / 2;
  size_t right_end = std::min(right_start + window - 1, num_readings - 1);
  
  double right_sum = 0.0;
  size_t right_count = 0;
  for (size_t i = right_start; i <= right_end; i++) {
    right_sum += last_scan_->ranges[i];
    ++right_count;
  }
  double right_avg = (right_count > 0) ? right_sum / right_count : 0.0;

  RCLCPP_INFO(this->get_logger(), "Left avg: %f, Right avg: %f", left_avg, right_avg);
  
  // Return +1 if left is clearer, -1 if right is clearer.
  return (left_avg > right_avg) ? +1 : -1;
}

} // namespace task1
