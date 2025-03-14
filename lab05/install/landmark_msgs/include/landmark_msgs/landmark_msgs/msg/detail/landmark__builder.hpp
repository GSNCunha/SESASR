// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from landmark_msgs:msg/Landmark.idl
// generated code does not contain a copyright notice

#ifndef LANDMARK_MSGS__MSG__DETAIL__LANDMARK__BUILDER_HPP_
#define LANDMARK_MSGS__MSG__DETAIL__LANDMARK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "landmark_msgs/msg/detail/landmark__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace landmark_msgs
{

namespace msg
{

namespace builder
{

class Init_Landmark_bearing
{
public:
  explicit Init_Landmark_bearing(::landmark_msgs::msg::Landmark & msg)
  : msg_(msg)
  {}
  ::landmark_msgs::msg::Landmark bearing(::landmark_msgs::msg::Landmark::_bearing_type arg)
  {
    msg_.bearing = std::move(arg);
    return std::move(msg_);
  }

private:
  ::landmark_msgs::msg::Landmark msg_;
};

class Init_Landmark_range
{
public:
  explicit Init_Landmark_range(::landmark_msgs::msg::Landmark & msg)
  : msg_(msg)
  {}
  Init_Landmark_bearing range(::landmark_msgs::msg::Landmark::_range_type arg)
  {
    msg_.range = std::move(arg);
    return Init_Landmark_bearing(msg_);
  }

private:
  ::landmark_msgs::msg::Landmark msg_;
};

class Init_Landmark_decision_margin
{
public:
  explicit Init_Landmark_decision_margin(::landmark_msgs::msg::Landmark & msg)
  : msg_(msg)
  {}
  Init_Landmark_range decision_margin(::landmark_msgs::msg::Landmark::_decision_margin_type arg)
  {
    msg_.decision_margin = std::move(arg);
    return Init_Landmark_range(msg_);
  }

private:
  ::landmark_msgs::msg::Landmark msg_;
};

class Init_Landmark_goodness
{
public:
  explicit Init_Landmark_goodness(::landmark_msgs::msg::Landmark & msg)
  : msg_(msg)
  {}
  Init_Landmark_decision_margin goodness(::landmark_msgs::msg::Landmark::_goodness_type arg)
  {
    msg_.goodness = std::move(arg);
    return Init_Landmark_decision_margin(msg_);
  }

private:
  ::landmark_msgs::msg::Landmark msg_;
};

class Init_Landmark_hamming
{
public:
  explicit Init_Landmark_hamming(::landmark_msgs::msg::Landmark & msg)
  : msg_(msg)
  {}
  Init_Landmark_goodness hamming(::landmark_msgs::msg::Landmark::_hamming_type arg)
  {
    msg_.hamming = std::move(arg);
    return Init_Landmark_goodness(msg_);
  }

private:
  ::landmark_msgs::msg::Landmark msg_;
};

class Init_Landmark_id
{
public:
  Init_Landmark_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Landmark_hamming id(::landmark_msgs::msg::Landmark::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_Landmark_hamming(msg_);
  }

private:
  ::landmark_msgs::msg::Landmark msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::landmark_msgs::msg::Landmark>()
{
  return landmark_msgs::msg::builder::Init_Landmark_id();
}

}  // namespace landmark_msgs

#endif  // LANDMARK_MSGS__MSG__DETAIL__LANDMARK__BUILDER_HPP_
