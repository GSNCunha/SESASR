// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from landmark_msgs:msg/Landmark.idl
// generated code does not contain a copyright notice

#ifndef LANDMARK_MSGS__MSG__DETAIL__LANDMARK__TRAITS_HPP_
#define LANDMARK_MSGS__MSG__DETAIL__LANDMARK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "landmark_msgs/msg/detail/landmark__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace landmark_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Landmark & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: hamming
  {
    out << "hamming: ";
    rosidl_generator_traits::value_to_yaml(msg.hamming, out);
    out << ", ";
  }

  // member: goodness
  {
    out << "goodness: ";
    rosidl_generator_traits::value_to_yaml(msg.goodness, out);
    out << ", ";
  }

  // member: decision_margin
  {
    out << "decision_margin: ";
    rosidl_generator_traits::value_to_yaml(msg.decision_margin, out);
    out << ", ";
  }

  // member: range
  {
    out << "range: ";
    rosidl_generator_traits::value_to_yaml(msg.range, out);
    out << ", ";
  }

  // member: bearing
  {
    out << "bearing: ";
    rosidl_generator_traits::value_to_yaml(msg.bearing, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Landmark & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: hamming
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hamming: ";
    rosidl_generator_traits::value_to_yaml(msg.hamming, out);
    out << "\n";
  }

  // member: goodness
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goodness: ";
    rosidl_generator_traits::value_to_yaml(msg.goodness, out);
    out << "\n";
  }

  // member: decision_margin
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "decision_margin: ";
    rosidl_generator_traits::value_to_yaml(msg.decision_margin, out);
    out << "\n";
  }

  // member: range
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "range: ";
    rosidl_generator_traits::value_to_yaml(msg.range, out);
    out << "\n";
  }

  // member: bearing
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bearing: ";
    rosidl_generator_traits::value_to_yaml(msg.bearing, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Landmark & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace landmark_msgs

namespace rosidl_generator_traits
{

[[deprecated("use landmark_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const landmark_msgs::msg::Landmark & msg,
  std::ostream & out, size_t indentation = 0)
{
  landmark_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use landmark_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const landmark_msgs::msg::Landmark & msg)
{
  return landmark_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<landmark_msgs::msg::Landmark>()
{
  return "landmark_msgs::msg::Landmark";
}

template<>
inline const char * name<landmark_msgs::msg::Landmark>()
{
  return "landmark_msgs/msg/Landmark";
}

template<>
struct has_fixed_size<landmark_msgs::msg::Landmark>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<landmark_msgs::msg::Landmark>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<landmark_msgs::msg::Landmark>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LANDMARK_MSGS__MSG__DETAIL__LANDMARK__TRAITS_HPP_
