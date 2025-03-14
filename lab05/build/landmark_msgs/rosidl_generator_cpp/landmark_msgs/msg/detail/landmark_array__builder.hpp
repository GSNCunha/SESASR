// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from landmark_msgs:msg/LandmarkArray.idl
// generated code does not contain a copyright notice

#ifndef LANDMARK_MSGS__MSG__DETAIL__LANDMARK_ARRAY__BUILDER_HPP_
#define LANDMARK_MSGS__MSG__DETAIL__LANDMARK_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "landmark_msgs/msg/detail/landmark_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace landmark_msgs
{

namespace msg
{

namespace builder
{

class Init_LandmarkArray_landmarks
{
public:
  explicit Init_LandmarkArray_landmarks(::landmark_msgs::msg::LandmarkArray & msg)
  : msg_(msg)
  {}
  ::landmark_msgs::msg::LandmarkArray landmarks(::landmark_msgs::msg::LandmarkArray::_landmarks_type arg)
  {
    msg_.landmarks = std::move(arg);
    return std::move(msg_);
  }

private:
  ::landmark_msgs::msg::LandmarkArray msg_;
};

class Init_LandmarkArray_header
{
public:
  Init_LandmarkArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LandmarkArray_landmarks header(::landmark_msgs::msg::LandmarkArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LandmarkArray_landmarks(msg_);
  }

private:
  ::landmark_msgs::msg::LandmarkArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::landmark_msgs::msg::LandmarkArray>()
{
  return landmark_msgs::msg::builder::Init_LandmarkArray_header();
}

}  // namespace landmark_msgs

#endif  // LANDMARK_MSGS__MSG__DETAIL__LANDMARK_ARRAY__BUILDER_HPP_
