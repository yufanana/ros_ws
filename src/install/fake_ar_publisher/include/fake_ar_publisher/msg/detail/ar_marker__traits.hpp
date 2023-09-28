// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from fake_ar_publisher:msg/ARMarker.idl
// generated code does not contain a copyright notice

#ifndef FAKE_AR_PUBLISHER__MSG__DETAIL__AR_MARKER__TRAITS_HPP_
#define FAKE_AR_PUBLISHER__MSG__DETAIL__AR_MARKER__TRAITS_HPP_

#include "fake_ar_publisher/msg/detail/ar_marker__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_with_covariance__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const fake_ar_publisher::msg::ARMarker & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_yaml(msg.header, out, indentation + 2);
  }

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_yaml(msg.pose, out, indentation + 2);
  }

  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence: ";
    value_to_yaml(msg.confidence, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const fake_ar_publisher::msg::ARMarker & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<fake_ar_publisher::msg::ARMarker>()
{
  return "fake_ar_publisher::msg::ARMarker";
}

template<>
inline const char * name<fake_ar_publisher::msg::ARMarker>()
{
  return "fake_ar_publisher/msg/ARMarker";
}

template<>
struct has_fixed_size<fake_ar_publisher::msg::ARMarker>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseWithCovariance>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<fake_ar_publisher::msg::ARMarker>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseWithCovariance>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<fake_ar_publisher::msg::ARMarker>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // FAKE_AR_PUBLISHER__MSG__DETAIL__AR_MARKER__TRAITS_HPP_
