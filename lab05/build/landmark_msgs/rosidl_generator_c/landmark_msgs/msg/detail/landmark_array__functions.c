// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from landmark_msgs:msg/LandmarkArray.idl
// generated code does not contain a copyright notice
#include "landmark_msgs/msg/detail/landmark_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `landmarks`
#include "landmark_msgs/msg/detail/landmark__functions.h"

bool
landmark_msgs__msg__LandmarkArray__init(landmark_msgs__msg__LandmarkArray * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    landmark_msgs__msg__LandmarkArray__fini(msg);
    return false;
  }
  // landmarks
  if (!landmark_msgs__msg__Landmark__Sequence__init(&msg->landmarks, 0)) {
    landmark_msgs__msg__LandmarkArray__fini(msg);
    return false;
  }
  return true;
}

void
landmark_msgs__msg__LandmarkArray__fini(landmark_msgs__msg__LandmarkArray * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // landmarks
  landmark_msgs__msg__Landmark__Sequence__fini(&msg->landmarks);
}

bool
landmark_msgs__msg__LandmarkArray__are_equal(const landmark_msgs__msg__LandmarkArray * lhs, const landmark_msgs__msg__LandmarkArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // landmarks
  if (!landmark_msgs__msg__Landmark__Sequence__are_equal(
      &(lhs->landmarks), &(rhs->landmarks)))
  {
    return false;
  }
  return true;
}

bool
landmark_msgs__msg__LandmarkArray__copy(
  const landmark_msgs__msg__LandmarkArray * input,
  landmark_msgs__msg__LandmarkArray * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // landmarks
  if (!landmark_msgs__msg__Landmark__Sequence__copy(
      &(input->landmarks), &(output->landmarks)))
  {
    return false;
  }
  return true;
}

landmark_msgs__msg__LandmarkArray *
landmark_msgs__msg__LandmarkArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  landmark_msgs__msg__LandmarkArray * msg = (landmark_msgs__msg__LandmarkArray *)allocator.allocate(sizeof(landmark_msgs__msg__LandmarkArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(landmark_msgs__msg__LandmarkArray));
  bool success = landmark_msgs__msg__LandmarkArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
landmark_msgs__msg__LandmarkArray__destroy(landmark_msgs__msg__LandmarkArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    landmark_msgs__msg__LandmarkArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
landmark_msgs__msg__LandmarkArray__Sequence__init(landmark_msgs__msg__LandmarkArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  landmark_msgs__msg__LandmarkArray * data = NULL;

  if (size) {
    data = (landmark_msgs__msg__LandmarkArray *)allocator.zero_allocate(size, sizeof(landmark_msgs__msg__LandmarkArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = landmark_msgs__msg__LandmarkArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        landmark_msgs__msg__LandmarkArray__fini(&data[i - 1]);
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
landmark_msgs__msg__LandmarkArray__Sequence__fini(landmark_msgs__msg__LandmarkArray__Sequence * array)
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
      landmark_msgs__msg__LandmarkArray__fini(&array->data[i]);
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

landmark_msgs__msg__LandmarkArray__Sequence *
landmark_msgs__msg__LandmarkArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  landmark_msgs__msg__LandmarkArray__Sequence * array = (landmark_msgs__msg__LandmarkArray__Sequence *)allocator.allocate(sizeof(landmark_msgs__msg__LandmarkArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = landmark_msgs__msg__LandmarkArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
landmark_msgs__msg__LandmarkArray__Sequence__destroy(landmark_msgs__msg__LandmarkArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    landmark_msgs__msg__LandmarkArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
landmark_msgs__msg__LandmarkArray__Sequence__are_equal(const landmark_msgs__msg__LandmarkArray__Sequence * lhs, const landmark_msgs__msg__LandmarkArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!landmark_msgs__msg__LandmarkArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
landmark_msgs__msg__LandmarkArray__Sequence__copy(
  const landmark_msgs__msg__LandmarkArray__Sequence * input,
  landmark_msgs__msg__LandmarkArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(landmark_msgs__msg__LandmarkArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    landmark_msgs__msg__LandmarkArray * data =
      (landmark_msgs__msg__LandmarkArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!landmark_msgs__msg__LandmarkArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          landmark_msgs__msg__LandmarkArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!landmark_msgs__msg__LandmarkArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
