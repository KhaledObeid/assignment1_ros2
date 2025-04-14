// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from br2_tracking_msgs:msg/PanTiltCommand.idl
// generated code does not contain a copyright notice

#ifndef BR2_TRACKING_MSGS__MSG__DETAIL__PAN_TILT_COMMAND__TRAITS_HPP_
#define BR2_TRACKING_MSGS__MSG__DETAIL__PAN_TILT_COMMAND__TRAITS_HPP_

#include "br2_tracking_msgs/msg/detail/pan_tilt_command__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<br2_tracking_msgs::msg::PanTiltCommand>()
{
  return "br2_tracking_msgs::msg::PanTiltCommand";
}

template<>
inline const char * name<br2_tracking_msgs::msg::PanTiltCommand>()
{
  return "br2_tracking_msgs/msg/PanTiltCommand";
}

template<>
struct has_fixed_size<br2_tracking_msgs::msg::PanTiltCommand>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<br2_tracking_msgs::msg::PanTiltCommand>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<br2_tracking_msgs::msg::PanTiltCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BR2_TRACKING_MSGS__MSG__DETAIL__PAN_TILT_COMMAND__TRAITS_HPP_
