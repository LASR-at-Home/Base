// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from lasr_vision_interfaces:srv/ClipLearnFace.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "lasr_vision_interfaces/srv/detail/clip_learn_face__rosidl_typesupport_introspection_c.h"
#include "lasr_vision_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "lasr_vision_interfaces/srv/detail/clip_learn_face__functions.h"
#include "lasr_vision_interfaces/srv/detail/clip_learn_face__struct.h"


// Include directives for member types
// Member `raw_imgs`
#include "sensor_msgs/msg/image.h"
// Member `raw_imgs`
#include "sensor_msgs/msg/detail/image__rosidl_typesupport_introspection_c.h"
// Member `name`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__ClipLearnFace_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  lasr_vision_interfaces__srv__ClipLearnFace_Request__init(message_memory);
}

void lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__ClipLearnFace_Request_fini_function(void * message_memory)
{
  lasr_vision_interfaces__srv__ClipLearnFace_Request__fini(message_memory);
}

size_t lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__size_function__ClipLearnFace_Request__raw_imgs(
  const void * untyped_member)
{
  const sensor_msgs__msg__Image__Sequence * member =
    (const sensor_msgs__msg__Image__Sequence *)(untyped_member);
  return member->size;
}

const void * lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__get_const_function__ClipLearnFace_Request__raw_imgs(
  const void * untyped_member, size_t index)
{
  const sensor_msgs__msg__Image__Sequence * member =
    (const sensor_msgs__msg__Image__Sequence *)(untyped_member);
  return &member->data[index];
}

void * lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__get_function__ClipLearnFace_Request__raw_imgs(
  void * untyped_member, size_t index)
{
  sensor_msgs__msg__Image__Sequence * member =
    (sensor_msgs__msg__Image__Sequence *)(untyped_member);
  return &member->data[index];
}

void lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__fetch_function__ClipLearnFace_Request__raw_imgs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const sensor_msgs__msg__Image * item =
    ((const sensor_msgs__msg__Image *)
    lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__get_const_function__ClipLearnFace_Request__raw_imgs(untyped_member, index));
  sensor_msgs__msg__Image * value =
    (sensor_msgs__msg__Image *)(untyped_value);
  *value = *item;
}

void lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__assign_function__ClipLearnFace_Request__raw_imgs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  sensor_msgs__msg__Image * item =
    ((sensor_msgs__msg__Image *)
    lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__get_function__ClipLearnFace_Request__raw_imgs(untyped_member, index));
  const sensor_msgs__msg__Image * value =
    (const sensor_msgs__msg__Image *)(untyped_value);
  *item = *value;
}

bool lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__resize_function__ClipLearnFace_Request__raw_imgs(
  void * untyped_member, size_t size)
{
  sensor_msgs__msg__Image__Sequence * member =
    (sensor_msgs__msg__Image__Sequence *)(untyped_member);
  sensor_msgs__msg__Image__Sequence__fini(member);
  return sensor_msgs__msg__Image__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__ClipLearnFace_Request_message_member_array[2] = {
  {
    "raw_imgs",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lasr_vision_interfaces__srv__ClipLearnFace_Request, raw_imgs),  // bytes offset in struct
    NULL,  // default value
    lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__size_function__ClipLearnFace_Request__raw_imgs,  // size() function pointer
    lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__get_const_function__ClipLearnFace_Request__raw_imgs,  // get_const(index) function pointer
    lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__get_function__ClipLearnFace_Request__raw_imgs,  // get(index) function pointer
    lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__fetch_function__ClipLearnFace_Request__raw_imgs,  // fetch(index, &value) function pointer
    lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__assign_function__ClipLearnFace_Request__raw_imgs,  // assign(index, value) function pointer
    lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__resize_function__ClipLearnFace_Request__raw_imgs  // resize(index) function pointer
  },
  {
    "name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lasr_vision_interfaces__srv__ClipLearnFace_Request, name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__ClipLearnFace_Request_message_members = {
  "lasr_vision_interfaces__srv",  // message namespace
  "ClipLearnFace_Request",  // message name
  2,  // number of fields
  sizeof(lasr_vision_interfaces__srv__ClipLearnFace_Request),
  lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__ClipLearnFace_Request_message_member_array,  // message members
  lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__ClipLearnFace_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__ClipLearnFace_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__ClipLearnFace_Request_message_type_support_handle = {
  0,
  &lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__ClipLearnFace_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_lasr_vision_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lasr_vision_interfaces, srv, ClipLearnFace_Request)() {
  lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__ClipLearnFace_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Image)();
  if (!lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__ClipLearnFace_Request_message_type_support_handle.typesupport_identifier) {
    lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__ClipLearnFace_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &lasr_vision_interfaces__srv__ClipLearnFace_Request__rosidl_typesupport_introspection_c__ClipLearnFace_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "lasr_vision_interfaces/srv/detail/clip_learn_face__rosidl_typesupport_introspection_c.h"
// already included above
// #include "lasr_vision_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "lasr_vision_interfaces/srv/detail/clip_learn_face__functions.h"
// already included above
// #include "lasr_vision_interfaces/srv/detail/clip_learn_face__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void lasr_vision_interfaces__srv__ClipLearnFace_Response__rosidl_typesupport_introspection_c__ClipLearnFace_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  lasr_vision_interfaces__srv__ClipLearnFace_Response__init(message_memory);
}

void lasr_vision_interfaces__srv__ClipLearnFace_Response__rosidl_typesupport_introspection_c__ClipLearnFace_Response_fini_function(void * message_memory)
{
  lasr_vision_interfaces__srv__ClipLearnFace_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember lasr_vision_interfaces__srv__ClipLearnFace_Response__rosidl_typesupport_introspection_c__ClipLearnFace_Response_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lasr_vision_interfaces__srv__ClipLearnFace_Response, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers lasr_vision_interfaces__srv__ClipLearnFace_Response__rosidl_typesupport_introspection_c__ClipLearnFace_Response_message_members = {
  "lasr_vision_interfaces__srv",  // message namespace
  "ClipLearnFace_Response",  // message name
  1,  // number of fields
  sizeof(lasr_vision_interfaces__srv__ClipLearnFace_Response),
  lasr_vision_interfaces__srv__ClipLearnFace_Response__rosidl_typesupport_introspection_c__ClipLearnFace_Response_message_member_array,  // message members
  lasr_vision_interfaces__srv__ClipLearnFace_Response__rosidl_typesupport_introspection_c__ClipLearnFace_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  lasr_vision_interfaces__srv__ClipLearnFace_Response__rosidl_typesupport_introspection_c__ClipLearnFace_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t lasr_vision_interfaces__srv__ClipLearnFace_Response__rosidl_typesupport_introspection_c__ClipLearnFace_Response_message_type_support_handle = {
  0,
  &lasr_vision_interfaces__srv__ClipLearnFace_Response__rosidl_typesupport_introspection_c__ClipLearnFace_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_lasr_vision_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lasr_vision_interfaces, srv, ClipLearnFace_Response)() {
  if (!lasr_vision_interfaces__srv__ClipLearnFace_Response__rosidl_typesupport_introspection_c__ClipLearnFace_Response_message_type_support_handle.typesupport_identifier) {
    lasr_vision_interfaces__srv__ClipLearnFace_Response__rosidl_typesupport_introspection_c__ClipLearnFace_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &lasr_vision_interfaces__srv__ClipLearnFace_Response__rosidl_typesupport_introspection_c__ClipLearnFace_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "lasr_vision_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "lasr_vision_interfaces/srv/detail/clip_learn_face__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers lasr_vision_interfaces__srv__detail__clip_learn_face__rosidl_typesupport_introspection_c__ClipLearnFace_service_members = {
  "lasr_vision_interfaces__srv",  // service namespace
  "ClipLearnFace",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // lasr_vision_interfaces__srv__detail__clip_learn_face__rosidl_typesupport_introspection_c__ClipLearnFace_Request_message_type_support_handle,
  NULL  // response message
  // lasr_vision_interfaces__srv__detail__clip_learn_face__rosidl_typesupport_introspection_c__ClipLearnFace_Response_message_type_support_handle
};

static rosidl_service_type_support_t lasr_vision_interfaces__srv__detail__clip_learn_face__rosidl_typesupport_introspection_c__ClipLearnFace_service_type_support_handle = {
  0,
  &lasr_vision_interfaces__srv__detail__clip_learn_face__rosidl_typesupport_introspection_c__ClipLearnFace_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lasr_vision_interfaces, srv, ClipLearnFace_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lasr_vision_interfaces, srv, ClipLearnFace_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_lasr_vision_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lasr_vision_interfaces, srv, ClipLearnFace)() {
  if (!lasr_vision_interfaces__srv__detail__clip_learn_face__rosidl_typesupport_introspection_c__ClipLearnFace_service_type_support_handle.typesupport_identifier) {
    lasr_vision_interfaces__srv__detail__clip_learn_face__rosidl_typesupport_introspection_c__ClipLearnFace_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)lasr_vision_interfaces__srv__detail__clip_learn_face__rosidl_typesupport_introspection_c__ClipLearnFace_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lasr_vision_interfaces, srv, ClipLearnFace_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lasr_vision_interfaces, srv, ClipLearnFace_Response)()->data;
  }

  return &lasr_vision_interfaces__srv__detail__clip_learn_face__rosidl_typesupport_introspection_c__ClipLearnFace_service_type_support_handle;
}
