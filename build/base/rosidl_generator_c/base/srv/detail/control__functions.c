// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from base:srv/Control.idl
// generated code does not contain a copyright notice
#include "base/srv/detail/control__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `topic`
// Member `func`
// Member `params`
#include "rosidl_runtime_c/string_functions.h"

bool
base__srv__Control_Request__init(base__srv__Control_Request * msg)
{
  if (!msg) {
    return false;
  }
  // topic
  if (!rosidl_runtime_c__String__init(&msg->topic)) {
    base__srv__Control_Request__fini(msg);
    return false;
  }
  // func
  if (!rosidl_runtime_c__String__init(&msg->func)) {
    base__srv__Control_Request__fini(msg);
    return false;
  }
  // flag
  // params
  if (!rosidl_runtime_c__String__init(&msg->params)) {
    base__srv__Control_Request__fini(msg);
    return false;
  }
  return true;
}

void
base__srv__Control_Request__fini(base__srv__Control_Request * msg)
{
  if (!msg) {
    return;
  }
  // topic
  rosidl_runtime_c__String__fini(&msg->topic);
  // func
  rosidl_runtime_c__String__fini(&msg->func);
  // flag
  // params
  rosidl_runtime_c__String__fini(&msg->params);
}

bool
base__srv__Control_Request__are_equal(const base__srv__Control_Request * lhs, const base__srv__Control_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // topic
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->topic), &(rhs->topic)))
  {
    return false;
  }
  // func
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->func), &(rhs->func)))
  {
    return false;
  }
  // flag
  if (lhs->flag != rhs->flag) {
    return false;
  }
  // params
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->params), &(rhs->params)))
  {
    return false;
  }
  return true;
}

bool
base__srv__Control_Request__copy(
  const base__srv__Control_Request * input,
  base__srv__Control_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // topic
  if (!rosidl_runtime_c__String__copy(
      &(input->topic), &(output->topic)))
  {
    return false;
  }
  // func
  if (!rosidl_runtime_c__String__copy(
      &(input->func), &(output->func)))
  {
    return false;
  }
  // flag
  output->flag = input->flag;
  // params
  if (!rosidl_runtime_c__String__copy(
      &(input->params), &(output->params)))
  {
    return false;
  }
  return true;
}

base__srv__Control_Request *
base__srv__Control_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base__srv__Control_Request * msg = (base__srv__Control_Request *)allocator.allocate(sizeof(base__srv__Control_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(base__srv__Control_Request));
  bool success = base__srv__Control_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
base__srv__Control_Request__destroy(base__srv__Control_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    base__srv__Control_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
base__srv__Control_Request__Sequence__init(base__srv__Control_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base__srv__Control_Request * data = NULL;

  if (size) {
    data = (base__srv__Control_Request *)allocator.zero_allocate(size, sizeof(base__srv__Control_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = base__srv__Control_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        base__srv__Control_Request__fini(&data[i - 1]);
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
base__srv__Control_Request__Sequence__fini(base__srv__Control_Request__Sequence * array)
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
      base__srv__Control_Request__fini(&array->data[i]);
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

base__srv__Control_Request__Sequence *
base__srv__Control_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base__srv__Control_Request__Sequence * array = (base__srv__Control_Request__Sequence *)allocator.allocate(sizeof(base__srv__Control_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = base__srv__Control_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
base__srv__Control_Request__Sequence__destroy(base__srv__Control_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    base__srv__Control_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
base__srv__Control_Request__Sequence__are_equal(const base__srv__Control_Request__Sequence * lhs, const base__srv__Control_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!base__srv__Control_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
base__srv__Control_Request__Sequence__copy(
  const base__srv__Control_Request__Sequence * input,
  base__srv__Control_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(base__srv__Control_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    base__srv__Control_Request * data =
      (base__srv__Control_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!base__srv__Control_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          base__srv__Control_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!base__srv__Control_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `value`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
base__srv__Control_Response__init(base__srv__Control_Response * msg)
{
  if (!msg) {
    return false;
  }
  // code
  // value
  if (!rosidl_runtime_c__String__init(&msg->value)) {
    base__srv__Control_Response__fini(msg);
    return false;
  }
  return true;
}

void
base__srv__Control_Response__fini(base__srv__Control_Response * msg)
{
  if (!msg) {
    return;
  }
  // code
  // value
  rosidl_runtime_c__String__fini(&msg->value);
}

bool
base__srv__Control_Response__are_equal(const base__srv__Control_Response * lhs, const base__srv__Control_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // code
  if (lhs->code != rhs->code) {
    return false;
  }
  // value
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->value), &(rhs->value)))
  {
    return false;
  }
  return true;
}

bool
base__srv__Control_Response__copy(
  const base__srv__Control_Response * input,
  base__srv__Control_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // code
  output->code = input->code;
  // value
  if (!rosidl_runtime_c__String__copy(
      &(input->value), &(output->value)))
  {
    return false;
  }
  return true;
}

base__srv__Control_Response *
base__srv__Control_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base__srv__Control_Response * msg = (base__srv__Control_Response *)allocator.allocate(sizeof(base__srv__Control_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(base__srv__Control_Response));
  bool success = base__srv__Control_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
base__srv__Control_Response__destroy(base__srv__Control_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    base__srv__Control_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
base__srv__Control_Response__Sequence__init(base__srv__Control_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base__srv__Control_Response * data = NULL;

  if (size) {
    data = (base__srv__Control_Response *)allocator.zero_allocate(size, sizeof(base__srv__Control_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = base__srv__Control_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        base__srv__Control_Response__fini(&data[i - 1]);
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
base__srv__Control_Response__Sequence__fini(base__srv__Control_Response__Sequence * array)
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
      base__srv__Control_Response__fini(&array->data[i]);
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

base__srv__Control_Response__Sequence *
base__srv__Control_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base__srv__Control_Response__Sequence * array = (base__srv__Control_Response__Sequence *)allocator.allocate(sizeof(base__srv__Control_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = base__srv__Control_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
base__srv__Control_Response__Sequence__destroy(base__srv__Control_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    base__srv__Control_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
base__srv__Control_Response__Sequence__are_equal(const base__srv__Control_Response__Sequence * lhs, const base__srv__Control_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!base__srv__Control_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
base__srv__Control_Response__Sequence__copy(
  const base__srv__Control_Response__Sequence * input,
  base__srv__Control_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(base__srv__Control_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    base__srv__Control_Response * data =
      (base__srv__Control_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!base__srv__Control_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          base__srv__Control_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!base__srv__Control_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
