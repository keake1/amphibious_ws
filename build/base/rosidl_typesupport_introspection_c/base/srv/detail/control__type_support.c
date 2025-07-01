// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from base:srv/Control.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "base/srv/detail/control__rosidl_typesupport_introspection_c.h"
#include "base/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "base/srv/detail/control__functions.h"
#include "base/srv/detail/control__struct.h"


// Include directives for member types
// Member `topic`
// Member `func`
// Member `params`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void base__srv__Control_Request__rosidl_typesupport_introspection_c__Control_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  base__srv__Control_Request__init(message_memory);
}

void base__srv__Control_Request__rosidl_typesupport_introspection_c__Control_Request_fini_function(void * message_memory)
{
  base__srv__Control_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember base__srv__Control_Request__rosidl_typesupport_introspection_c__Control_Request_message_member_array[4] = {
  {
    "topic",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(base__srv__Control_Request, topic),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "func",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(base__srv__Control_Request, func),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "flag",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(base__srv__Control_Request, flag),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "params",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(base__srv__Control_Request, params),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers base__srv__Control_Request__rosidl_typesupport_introspection_c__Control_Request_message_members = {
  "base__srv",  // message namespace
  "Control_Request",  // message name
  4,  // number of fields
  sizeof(base__srv__Control_Request),
  base__srv__Control_Request__rosidl_typesupport_introspection_c__Control_Request_message_member_array,  // message members
  base__srv__Control_Request__rosidl_typesupport_introspection_c__Control_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  base__srv__Control_Request__rosidl_typesupport_introspection_c__Control_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t base__srv__Control_Request__rosidl_typesupport_introspection_c__Control_Request_message_type_support_handle = {
  0,
  &base__srv__Control_Request__rosidl_typesupport_introspection_c__Control_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_base
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, base, srv, Control_Request)() {
  if (!base__srv__Control_Request__rosidl_typesupport_introspection_c__Control_Request_message_type_support_handle.typesupport_identifier) {
    base__srv__Control_Request__rosidl_typesupport_introspection_c__Control_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &base__srv__Control_Request__rosidl_typesupport_introspection_c__Control_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "base/srv/detail/control__rosidl_typesupport_introspection_c.h"
// already included above
// #include "base/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "base/srv/detail/control__functions.h"
// already included above
// #include "base/srv/detail/control__struct.h"


// Include directives for member types
// Member `value`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void base__srv__Control_Response__rosidl_typesupport_introspection_c__Control_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  base__srv__Control_Response__init(message_memory);
}

void base__srv__Control_Response__rosidl_typesupport_introspection_c__Control_Response_fini_function(void * message_memory)
{
  base__srv__Control_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember base__srv__Control_Response__rosidl_typesupport_introspection_c__Control_Response_message_member_array[2] = {
  {
    "code",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(base__srv__Control_Response, code),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "value",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(base__srv__Control_Response, value),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers base__srv__Control_Response__rosidl_typesupport_introspection_c__Control_Response_message_members = {
  "base__srv",  // message namespace
  "Control_Response",  // message name
  2,  // number of fields
  sizeof(base__srv__Control_Response),
  base__srv__Control_Response__rosidl_typesupport_introspection_c__Control_Response_message_member_array,  // message members
  base__srv__Control_Response__rosidl_typesupport_introspection_c__Control_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  base__srv__Control_Response__rosidl_typesupport_introspection_c__Control_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t base__srv__Control_Response__rosidl_typesupport_introspection_c__Control_Response_message_type_support_handle = {
  0,
  &base__srv__Control_Response__rosidl_typesupport_introspection_c__Control_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_base
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, base, srv, Control_Response)() {
  if (!base__srv__Control_Response__rosidl_typesupport_introspection_c__Control_Response_message_type_support_handle.typesupport_identifier) {
    base__srv__Control_Response__rosidl_typesupport_introspection_c__Control_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &base__srv__Control_Response__rosidl_typesupport_introspection_c__Control_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "base/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "base/srv/detail/control__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers base__srv__detail__control__rosidl_typesupport_introspection_c__Control_service_members = {
  "base__srv",  // service namespace
  "Control",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // base__srv__detail__control__rosidl_typesupport_introspection_c__Control_Request_message_type_support_handle,
  NULL  // response message
  // base__srv__detail__control__rosidl_typesupport_introspection_c__Control_Response_message_type_support_handle
};

static rosidl_service_type_support_t base__srv__detail__control__rosidl_typesupport_introspection_c__Control_service_type_support_handle = {
  0,
  &base__srv__detail__control__rosidl_typesupport_introspection_c__Control_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, base, srv, Control_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, base, srv, Control_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_base
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, base, srv, Control)() {
  if (!base__srv__detail__control__rosidl_typesupport_introspection_c__Control_service_type_support_handle.typesupport_identifier) {
    base__srv__detail__control__rosidl_typesupport_introspection_c__Control_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)base__srv__detail__control__rosidl_typesupport_introspection_c__Control_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, base, srv, Control_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, base, srv, Control_Response)()->data;
  }

  return &base__srv__detail__control__rosidl_typesupport_introspection_c__Control_service_type_support_handle;
}
