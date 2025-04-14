#include <utility>
#include <cmath> // for M_PI, fabs
#include "t2b/BumpGoNode.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

// Define required constants (adjust values as needed):
#define OBSTACLE_DISTANCE 0.5
#define SCAN_TIMEOUT 1.0              // seconds for sensor timeout
#define SPEED_LINEAR 0.2              // m/s forward speed
#define SPEED_ANGULAR 0.5             // rad/s (maximum angular command)
#define BACKING_TIME 2.0              // seconds to remain in BACK state

// Angular region definitions (in radians):
// We use lateral sectors from +5° to +90° (left) and -90° to -5° (right).
#define LEFT_SECTOR_MIN (0.0873)      // +5° in radians
#define LEFT_SECTOR_MAX (M_PI / 2)      // 90° in radians
#define RIGHT_SECTOR_MIN (-M_PI / 2)    // -90° in radians
#define RIGHT_SECTOR_MAX (-0.0873)     // -5° in radians

// New definitions for stuck detection:
#define STUCK_TIMEOUT 2.0    // seconds: if best free-space readings remain nearly unchanged for this long, assume stuck.
#define STUCK_EPSILON 0.1    // acceptable difference in range (m) and angle (rad) between cycles.
#define STUCK_BACK_TIME 3.0  // seconds to back up when stuck.
#define STUCK 4              // New state value for STUCK.

namespace t2b
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
  // Do nothing until the first sensor read.
  if (last_scan_ == nullptr) {
    return;
  }

  geometry_msgs::msg::Twist out_vel;

  switch (state_) {
    case FORWARD:
    {
      // Set forward linear speed.
      out_vel.linear.x = SPEED_LINEAR;

      // --- Stuck Detection ---
      // Compute the farthest free space from the lateral sectors (left: +5° to +90°, right: -90° to -5°).
      double best_range = -1.0;
      double best_angle = 0.0;
      double angle = last_scan_->angle_min;
      for (size_t i = 0; i < last_scan_->ranges.size(); i++) {
        if ((angle >= LEFT_SECTOR_MIN && angle <= LEFT_SECTOR_MAX) ||
            (angle >= RIGHT_SECTOR_MIN && angle <= RIGHT_SECTOR_MAX)) {
          double range = last_scan_->ranges[i];
          if (range > best_range) {
            best_range = range;
            best_angle = angle;
          }
        }
        angle += last_scan_->angle_increment;
      }

      // Log the best free-space information.
      RCLCPP_INFO(get_logger(), "Best free space: range = %.2f m, angle = %.2f rad", best_range, best_angle);

      // --- Stuck Detection Logic ---
      // Compare the current best_range and best_angle to those of the previous cycle.
      static bool first_cycle = true;
      static double last_best_range = 0.0;
      static double last_best_angle = 0.0;
      static rclcpp::Time stuck_start_time = now();
      if (first_cycle) {
        last_best_range = best_range;
        last_best_angle = best_angle;
        stuck_start_time = now();
        first_cycle = false;
      } else {
        // If the difference in both range and angle is below a threshold, consider readings unchanged.
        if (std::fabs(best_range - last_best_range) < STUCK_EPSILON &&
            std::fabs(best_angle - last_best_angle) < STUCK_EPSILON)
        {
          if ((now() - stuck_start_time) > rclcpp::Duration::from_seconds(STUCK_TIMEOUT)) {
            RCLCPP_WARN(get_logger(), "Stuck detected. Initiating recovery.");
            go_state(STUCK);
          }
        } else {
          // Readings have changed; reset the stuck timer.
          stuck_start_time = now();
          last_best_range = best_range;
          last_best_angle = best_angle;
        }
      }

      // Use a proportional controller to adjust heading toward the detected free space.
      constexpr double ANGLE_THRESHOLD = 0.05; // rad; below which no correction is applied.
      double Kp = 1.0; // Proportional gain.
      if (std::fabs(best_angle) > ANGLE_THRESHOLD) {
        out_vel.angular.z = Kp * best_angle;
        // Clamp the angular velocity.
        if (out_vel.angular.z > SPEED_ANGULAR)
          out_vel.angular.z = SPEED_ANGULAR;
        else if (out_vel.angular.z < -SPEED_ANGULAR)
          out_vel.angular.z = -SPEED_ANGULAR;
      } else {
        out_vel.angular.z = 0.0;
      }

      // Check for sensor timeout or normal obstacle conditions.
      if (check_forward_2_stop()) {
        go_state(STOP);
      }
      if (check_forward_2_back()) {
        go_state(BACK);
      }
      break;
    }

    case BACK:
      out_vel.linear.x = -SPEED_LINEAR;
      if (check_back_2_turn()) {
        go_state(TURN);
      }
      break;

    case TURN:
      // In this design, turning correction is handled in the FORWARD state.
      break;

    case STOP:
      if (check_stop_2_forward()) {
        go_state(FORWARD);
      }
      break;

    case STUCK: {
      // In the STUCK state, the robot backs up for a fixed time, then transitions to TURN.
      out_vel.linear.x = -SPEED_LINEAR;
      out_vel.angular.z = 0.0;
      if ((now() - state_ts_) > rclcpp::Duration::from_seconds(STUCK_BACK_TIME)) {
        go_state(TURN);
      }
      break;
    }

    default:
      break;
  }

  vel_pub_->publish(out_vel);
}

void
BumpGoNode::go_state(int new_state)
{
  state_ = new_state;
  state_ts_ = now();

  // Optionally, reset any state-specific static variables here if needed.
  // For stuck detection in FORWARD state, we reset on leaving FORWARD.
  if (new_state != FORWARD) {
    // Reset stuck detection variables on state change if desired.
    // (They are static inside control_cycle and will be reinitialized when FORWARD is reentered.)
  }
}

bool
BumpGoNode::check_forward_2_back()
{
  // Use the central beam (approximately) as a simple check.
  size_t center = last_scan_->ranges.size() / 2;
  return last_scan_->ranges[center] < OBSTACLE_DISTANCE;
}

bool
BumpGoNode::check_forward_2_stop()
{
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed > rclcpp::Duration::from_seconds(SCAN_TIMEOUT);
}

bool
BumpGoNode::check_stop_2_forward()
{
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed < rclcpp::Duration::from_seconds(SCAN_TIMEOUT);
}

bool
BumpGoNode::check_back_2_turn()
{
  return (now() - state_ts_) > rclcpp::Duration::from_seconds(BACKING_TIME);
}

bool
BumpGoNode::check_turn_2_forward()
{
  return false;
}

}  // namespace t2b
