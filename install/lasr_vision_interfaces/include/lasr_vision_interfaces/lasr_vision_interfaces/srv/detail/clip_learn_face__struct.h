// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from lasr_vision_interfaces:srv/ClipLearnFace.idl
// generated code does not contain a copyright notice

#ifndef LASR_VISION_INTERFACES__SRV__DETAIL__CLIP_LEARN_FACE__STRUCT_H_
#define LASR_VISION_INTERFACES__SRV__DETAIL__CLIP_LEARN_FACE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'raw_imgs'
#include "sensor_msgs/msg/detail/image__struct.h"
// Member 'name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ClipLearnFace in the package lasr_vision_interfaces.
typedef struct lasr_vision_interfaces__srv__ClipLearnFace_Request
{
  sensor_msgs__msg__Image__Sequence raw_imgs;
  /// Name of person to be learned
  rosidl_runtime_c__String name;
} lasr_vision_interfaces__srv__ClipLearnFace_Request;

// Struct for a sequence of lasr_vision_interfaces__srv__ClipLearnFace_Request.
typedef struct lasr_vision_interfaces__srv__ClipLearnFace_Request__Sequence
{
  lasr_vision_interfaces__srv__ClipLearnFace_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lasr_vision_interfaces__srv__ClipLearnFace_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ClipLearnFace in the package lasr_vision_interfaces.
typedef struct lasr_vision_interfaces__srv__ClipLearnFace_Response
{
  uint8_t structure_needs_at_least_one_member;
} lasr_vision_interfaces__srv__ClipLearnFace_Response;

// Struct for a sequence of lasr_vision_interfaces__srv__ClipLearnFace_Response.
typedef struct lasr_vision_interfaces__srv__ClipLearnFace_Response__Sequence
{
  lasr_vision_interfaces__srv__ClipLearnFace_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lasr_vision_interfaces__srv__ClipLearnFace_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LASR_VISION_INTERFACES__SRV__DETAIL__CLIP_LEARN_FACE__STRUCT_H_
