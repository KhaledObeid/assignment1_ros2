#include <utility>
#include <algorithm>
#include <limits>
#include "BumpGoNode2b.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace task2b {

using namespace std::chrono_literals;
using std::placeholders::_1;

// Define static Duration members.
const rclcpp::Duration BumpGoNode2b::SCAN_TIMEOUT = rclcpp::Duration::from_seconds(0.5);
const rclcpp::Duration BumpGoNode2b::BACKING_TIME  = rclcpp::Duration::from_seconds(1.0);

BumpGoNode2b::BumpGoNode2b()
: Node("bump_go"),
  state_(FORWARD),
  turn_direction_(+1)
{
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan",
    rclcpp::SensorDataQoS(),
    std::bind(&BumpGoNode2b::scan_callback, this, _1)
  );

  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
  timer_ = create_wall_timer(50ms, std::bind(&BumpGoNode2b::control_cycle, this));
  
  state_ts_ = now();
}

void BumpGoNode2b::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  last_scan_ = std::move(msg);
}

void BumpGoNode2b::control_cycle()
{
  if (last_scan_ == nullptr)
    return;  // Wait for sensor data

  geometry_msgs::msg::Twist out_vel;

  switch (state_) {
    case FORWARD: {
      out_vel.linear.x = SPEED_LINEAR;
      // Use the front region from -45° to +45° for detecting obstacles.
      double front_clearance = get_region_clearance(-45.0 * M_PI/180.0, 45.0 * M_PI/180.0);
      if (front_clearance < OBSTACLE_DISTANCE) {
        // Compute clearances for left and right using broader regions:
        double left_clearance  = get_region_clearance(-120.0 * M_PI/180.0, -45.0 * M_PI/180.0);
        double right_clearance = get_region_clearance(45.0 * M_PI/180.0, 120.0 * M_PI/180.0);
        RCLCPP_INFO(get_logger(),
                    "Obstacle detected! Front clearance: %.2f m (threshold: %.2f m)",
                    front_clearance, OBSTACLE_DISTANCE);
        RCLCPP_INFO(get_logger(), "Left clearance: %.2f m, Right clearance: %.2f m",
                    left_clearance, right_clearance);
        // Decide turn direction based on which side is clearer.
        turn_direction_ = (left_clearance > right_clearance) ? +1 : -1;
        go_state(BACK);
      }
      break;
    }
    case BACK: {
      out_vel.linear.x = -SPEED_LINEAR;
      if (check_back_2_turn())
        go_state(TURN);
      break;
    }
    case TURN: {
      out_vel.angular.z = turn_direction_ * TURN_ANGULAR;
      // For exiting TURN, use the average clearance over a narrow front window.
      double narrow_front = get_region_clearance(-15.0 * M_PI/180.0, 15.0 * M_PI/180.0);
      RCLCPP_INFO(get_logger(), "TURNING: Narrow front clearance: %.2f m (threshold: %.2f m)",
                  narrow_front, CLEAR_THRESHOLD);
      if (narrow_front > CLEAR_THRESHOLD) {
        RCLCPP_INFO(get_logger(), "Path is clear. Transitioning to FORWARD.");
        go_state(FORWARD);
      }
      break;
    }
    case STOP: {
      if (check_stop_2_forward())
        go_state(FORWARD);
      break;
    }
  }

  vel_pub_->publish(out_vel);
}

void BumpGoNode2b::go_state(int new_state)
{
  state_ = new_state;
  state_ts_ = now();
}

bool BumpGoNode2b::check_forward_2_back()
{
  // Check the central reading for obstacles.
  size_t center_index = last_scan_->ranges.size() / 2;
  return last_scan_->ranges[center_index] < OBSTACLE_DISTANCE;
}

bool BumpGoNode2b::check_forward_2_stop()
{
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed > SCAN_TIMEOUT;
}

bool BumpGoNode2b::check_stop_2_forward()
{
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed < SCAN_TIMEOUT;
}

bool BumpGoNode2b::check_back_2_turn()
{
  return (now() - state_ts_) > BACKING_TIME;
}

// For closed-loop, we exit TURN when the narrow front clearance exceeds CLEAR_THRESHOLD.
bool BumpGoNode2b::check_turn_2_forward()
{
  double narrow_front = get_region_clearance(-15.0 * M_PI/180.0, 15.0 * M_PI/180.0);
  return narrow_front > CLEAR_THRESHOLD;
}

// Helper: compute average clearance in a specified angular region.
double BumpGoNode2b::get_region_clearance(double angle_start, double angle_end)
{
  if (!last_scan_) return 0.0;
  
  // Convert angles in radians to indices.
  double angleMin = last_scan_->angle_min;
  double inc      = last_scan_->angle_increment;
  size_t index_start = static_cast<size_t>((angle_start - angleMin) / inc);
  size_t index_end   = static_cast<size_t>((angle_end   - angleMin) / inc);
  
  index_start = std::max(index_start, size_t(0));
  index_end = std::min(index_end, last_scan_->ranges.size() - 1);
  
  double sum = 0.0;
  int count = 0;
  for (size_t i = index_start; i <= index_end; ++i) {
    sum += last_scan_->ranges[i];
    count++;
  }
  return (count > 0) ? sum / count : 0.0;
}

} // namespace task2b
