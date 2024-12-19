// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from landmark_msgs:msg/LandmarkArray.idl
// generated code does not contain a copyright notice

#ifndef LANDMARK_MSGS__MSG__DETAIL__LANDMARK_ARRAY__STRUCT_H_
#define LANDMARK_MSGS__MSG__DETAIL__LANDMARK_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'landmarks'
#include "landmark_msgs/msg/detail/landmark__struct.h"

/// Struct defined in msg/LandmarkArray in the package landmark_msgs.
typedef struct landmark_msgs__msg__LandmarkArray
{
  std_msgs__msg__Header header;
  landmark_msgs__msg__Landmark__Sequence landmarks;
} landmark_msgs__msg__LandmarkArray;

// Struct for a sequence of landmark_msgs__msg__LandmarkArray.
typedef struct landmark_msgs__msg__LandmarkArray__Sequence
{
  landmark_msgs__msg__LandmarkArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} landmark_msgs__msg__LandmarkArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LANDMARK_MSGS__MSG__DETAIL__LANDMARK_ARRAY__STRUCT_H_
