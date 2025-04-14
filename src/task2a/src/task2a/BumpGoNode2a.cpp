#include <utility>
#include <algorithm>
#include <limits>
#include "BumpGoNode2a.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace task2a {

using namespace std::chrono_literals;
using std::placeholders::_1;

// Define the static const Duration members.
const rclcpp::Duration BumpGoNode2a::SCAN_TIMEOUT  = rclcpp::Duration::from_seconds(0.5);
const rclcpp::Duration BumpGoNode2a::BACKING_TIME   = rclcpp::Duration::from_seconds(1.0);
const rclcpp::Duration BumpGoNode2a::TURNING_TIME   = rclcpp::Duration::from_seconds(3.0);

BumpGoNode2a::BumpGoNode2a()
: Node("bump_go"),
  state_(FORWARD),
  turn_direction_(+1),
  target_turn_angle_(0.0),
  turn_duration_(0.0),
  computed_angular_speed_(0.5)
{
  // Subscribe to the laser scan topic.
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan",
    rclcpp::SensorDataQoS(),
    std::bind(&BumpGoNode2a::scan_callback, this, _1)
  );

  // Publisher for velocity commands.
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);

  // Create a timer for the control cycle (20 Hz).
  timer_ = create_wall_timer(50ms, std::bind(&BumpGoNode2a::control_cycle, this));

  state_ts_ = now();
}

void BumpGoNode2a::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  last_scan_ = std::move(msg);
}

void BumpGoNode2a::control_cycle()
{
  if (last_scan_ == nullptr)
    return;  // Wait until we have valid sensor data

  geometry_msgs::msg::Twist out_vel;

  switch (state_) {
    case FORWARD:
      out_vel.linear.x = SPEED_LINEAR;
      if (check_forward_2_stop()) {
        go_state(STOP);
      } else if (check_forward_2_back()) {
        // Compute target turn angle with open-loop approach.
        double angle = compute_target_turn_angle();
        // Decide turn direction based on the sign of 'angle'
        turn_direction_ = (angle >= 0.0) ? +1 : -1;
        target_turn_angle_ = std::abs(angle);
        // Dynamically compute angular speed, here proportional to target angle.
        double k = 1.0;
        computed_angular_speed_ = k * target_turn_angle_;
        // Clamp computed speed between MIN_ANGULAR_SPEED and MAX_ANGULAR_SPEED.
        computed_angular_speed_ = std::min(MAX_ANGULAR_SPEED,
                                   std::max(MIN_ANGULAR_SPEED, computed_angular_speed_));
        // Compute the turn duration (t = angle / angular speed).
        turn_duration_ = target_turn_angle_ / computed_angular_speed_;
        RCLCPP_INFO(get_logger(),
                    "Computed turn angle: %f rad, angular speed: %f rad/s, turn duration: %f s",
                    angle, computed_angular_speed_, turn_duration_);
        go_state(BACK);
      }
      break;

    case BACK:
      out_vel.linear.x = -SPEED_LINEAR;
      if (check_back_2_turn())
        go_state(TURN);
      break;

    case TURN:
      // Use the computed angular speed.
      out_vel.angular.z = turn_direction_ * computed_angular_speed_;
      // Exit TURN state once the elapsed time exceeds the computed turn duration.
      if ((now() - state_ts_) >= rclcpp::Duration::from_seconds(turn_duration_))
        go_state(FORWARD);
      break;

    case STOP:
      if (check_stop_2_forward())
        go_state(FORWARD);
      break;
  }

  vel_pub_->publish(out_vel);
}

void BumpGoNode2a::go_state(int new_state)
{
  state_ = new_state;
  state_ts_ = now();
}

bool BumpGoNode2a::check_forward_2_back()
{
  size_t center_index = last_scan_->ranges.size() / 2;
  return last_scan_->ranges[center_index] < OBSTACLE_DISTANCE;
}

bool BumpGoNode2a::check_forward_2_stop()
{
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed > SCAN_TIMEOUT;
}

bool BumpGoNode2a::check_stop_2_forward()
{
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed < SCAN_TIMEOUT;
}

bool BumpGoNode2a::check_back_2_turn()
{
  return (now() - state_ts_) > BACKING_TIME;
}

// For open-loop turning, we exit TURN state based solely on elapsed time.
bool BumpGoNode2a::check_turn_2_forward()
{
  return false;
}

double BumpGoNode2a::compute_target_turn_angle()
{
  // Find the index with the maximum range (i.e. the clearest direction).
  double max_range = -std::numeric_limits<double>::infinity();
  size_t best_index = last_scan_->ranges.size() / 2;  // default to center
  for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
    double current_range = last_scan_->ranges[i];
    if (current_range > max_range) {
      max_range = current_range;
      best_index = i;
    }
  }
  // Compute the corresponding angle from the scan data.
  double target_angle = last_scan_->angle_min + best_index * last_scan_->angle_increment;
  return target_angle;
}

} // namespace task2a
