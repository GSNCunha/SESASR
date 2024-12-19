// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from landmark_msgs:msg/Landmark.idl
// generated code does not contain a copyright notice
#include "landmark_msgs/msg/detail/landmark__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
landmark_msgs__msg__Landmark__init(landmark_msgs__msg__Landmark * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // hamming
  msg->hamming = 0l;
  // goodness
  msg->goodness = 1.0f;
  // decision_margin
  msg->decision_margin = 1.0f;
  // range
  // bearing
  return true;
}

void
landmark_msgs__msg__Landmark__fini(landmark_msgs__msg__Landmark * msg)
{
  if (!msg) {
    return;
  }
  // id
  // hamming
  // goodness
  // decision_margin
  // range
  // bearing
}

bool
landmark_msgs__msg__Landmark__are_equal(const landmark_msgs__msg__Landmark * lhs, const landmark_msgs__msg__Landmark * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // hamming
  if (lhs->hamming != rhs->hamming) {
    return false;
  }
  // goodness
  if (lhs->goodness != rhs->goodness) {
    return false;
  }
  // decision_margin
  if (lhs->decision_margin != rhs->decision_margin) {
    return false;
  }
  // range
  if (lhs->range != rhs->range) {
    return false;
  }
  // bearing
  if (lhs->bearing != rhs->bearing) {
    return false;
  }
  return true;
}

bool
landmark_msgs__msg__Landmark__copy(
  const landmark_msgs__msg__Landmark * input,
  landmark_msgs__msg__Landmark * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // hamming
  output->hamming = input->hamming;
  // goodness
  output->goodness = input->goodness;
  // decision_margin
  output->decision_margin = input->decision_margin;
  // range
  output->range = input->range;
  // bearing
  output->bearing = input->bearing;
  return true;
}

landmark_msgs__msg__Landmark *
landmark_msgs__msg__Landmark__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  landmark_msgs__msg__Landmark * msg = (landmark_msgs__msg__Landmark *)allocator.allocate(sizeof(landmark_msgs__msg__Landmark), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(landmark_msgs__msg__Landmark));
  bool success = landmark_msgs__msg__Landmark__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
landmark_msgs__msg__Landmark__destroy(landmark_msgs__msg__Landmark * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    landmark_msgs__msg__Landmark__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
landmark_msgs__msg__Landmark__Sequence__init(landmark_msgs__msg__Landmark__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  landmark_msgs__msg__Landmark * data = NULL;

  if (size) {
    data = (landmark_msgs__msg__Landmark *)allocator.zero_allocate(size, sizeof(landmark_msgs__msg__Landmark), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = landmark_msgs__msg__Landmark__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        landmark_msgs__msg__Landmark__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
landmark_msgs__msg__Landmark__Sequence__fini(landmark_msgs__msg__Landmark__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      landmark_msgs__msg__Landmark__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

landmark_msgs__msg__Landmark__Sequence *
landmark_msgs__msg__Landmark__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  landmark_msgs__msg__Landmark__Sequence * array = (landmark_msgs__msg__Landmark__Sequence *)allocator.allocate(sizeof(landmark_msgs__msg__Landmark__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = landmark_msgs__msg__Landmark__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
landmark_msgs__msg__Landmark__Sequence__destroy(landmark_msgs__msg__Landmark__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    landmark_msgs__msg__Landmark__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
landmark_msgs__msg__Landmark__Sequence__are_equal(const landmark_msgs__msg__Landmark__Sequence * lhs, const landmark_msgs__msg__Landmark__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!landmark_msgs__msg__Landmark__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
landmark_msgs__msg__Landmark__Sequence__copy(
  const landmark_msgs__msg__Landmark__Sequence * input,
  landmark_msgs__msg__Landmark__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(landmark_msgs__msg__Landmark);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    landmark_msgs__msg__Landmark * data =
      (landmark_msgs__msg__Landmark *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!landmark_msgs__msg__Landmark__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          landmark_msgs__msg__Landmark__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!landmark_msgs__msg__Landmark__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
