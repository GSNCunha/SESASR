// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from landmark_msgs:msg/Landmark.idl
// generated code does not contain a copyright notice

#ifndef LANDMARK_MSGS__MSG__DETAIL__LANDMARK__STRUCT_H_
#define LANDMARK_MSGS__MSG__DETAIL__LANDMARK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Landmark in the package landmark_msgs.
typedef struct landmark_msgs__msg__Landmark
{
  int32_t id;
  int32_t hamming;
  float goodness;
  float decision_margin;
  float range;
  float bearing;
} landmark_msgs__msg__Landmark;

// Struct for a sequence of landmark_msgs__msg__Landmark.
typedef struct landmark_msgs__msg__Landmark__Sequence
{
  landmark_msgs__msg__Landmark * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} landmark_msgs__msg__Landmark__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LANDMARK_MSGS__MSG__DETAIL__LANDMARK__STRUCT_H_
