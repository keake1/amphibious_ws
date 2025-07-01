// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from base:srv/Control.idl
// generated code does not contain a copyright notice

#ifndef BASE__SRV__DETAIL__CONTROL__STRUCT_H_
#define BASE__SRV__DETAIL__CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'topic'
// Member 'func'
// Member 'params'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Control in the package base.
typedef struct base__srv__Control_Request
{
  rosidl_runtime_c__String topic;
  rosidl_runtime_c__String func;
  int8_t flag;
  rosidl_runtime_c__String params;
} base__srv__Control_Request;

// Struct for a sequence of base__srv__Control_Request.
typedef struct base__srv__Control_Request__Sequence
{
  base__srv__Control_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} base__srv__Control_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'value'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Control in the package base.
typedef struct base__srv__Control_Response
{
  int32_t code;
  rosidl_runtime_c__String value;
} base__srv__Control_Response;

// Struct for a sequence of base__srv__Control_Response.
typedef struct base__srv__Control_Response__Sequence
{
  base__srv__Control_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} base__srv__Control_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BASE__SRV__DETAIL__CONTROL__STRUCT_H_
