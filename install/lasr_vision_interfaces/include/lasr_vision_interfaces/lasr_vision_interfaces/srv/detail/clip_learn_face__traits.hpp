// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from lasr_vision_interfaces:srv/ClipLearnFace.idl
// generated code does not contain a copyright notice

#ifndef LASR_VISION_INTERFACES__SRV__DETAIL__CLIP_LEARN_FACE__TRAITS_HPP_
#define LASR_VISION_INTERFACES__SRV__DETAIL__CLIP_LEARN_FACE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "lasr_vision_interfaces/srv/detail/clip_learn_face__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'raw_imgs'
#include "sensor_msgs/msg/detail/image__traits.hpp"

namespace lasr_vision_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ClipLearnFace_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: raw_imgs
  {
    if (msg.raw_imgs.size() == 0) {
      out << "raw_imgs: []";
    } else {
      out << "raw_imgs: [";
      size_t pending_items = msg.raw_imgs.size();
      for (auto item : msg.raw_imgs) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ClipLearnFace_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: raw_imgs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.raw_imgs.size() == 0) {
      out << "raw_imgs: []\n";
    } else {
      out << "raw_imgs:\n";
      for (auto item : msg.raw_imgs) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ClipLearnFace_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace lasr_vision_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use lasr_vision_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const lasr_vision_interfaces::srv::ClipLearnFace_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  lasr_vision_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use lasr_vision_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const lasr_vision_interfaces::srv::ClipLearnFace_Request & msg)
{
  return lasr_vision_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<lasr_vision_interfaces::srv::ClipLearnFace_Request>()
{
  return "lasr_vision_interfaces::srv::ClipLearnFace_Request";
}

template<>
inline const char * name<lasr_vision_interfaces::srv::ClipLearnFace_Request>()
{
  return "lasr_vision_interfaces/srv/ClipLearnFace_Request";
}

template<>
struct has_fixed_size<lasr_vision_interfaces::srv::ClipLearnFace_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<lasr_vision_interfaces::srv::ClipLearnFace_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<lasr_vision_interfaces::srv::ClipLearnFace_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace lasr_vision_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ClipLearnFace_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ClipLearnFace_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ClipLearnFace_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace lasr_vision_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use lasr_vision_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const lasr_vision_interfaces::srv::ClipLearnFace_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  lasr_vision_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use lasr_vision_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const lasr_vision_interfaces::srv::ClipLearnFace_Response & msg)
{
  return lasr_vision_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<lasr_vision_interfaces::srv::ClipLearnFace_Response>()
{
  return "lasr_vision_interfaces::srv::ClipLearnFace_Response";
}

template<>
inline const char * name<lasr_vision_interfaces::srv::ClipLearnFace_Response>()
{
  return "lasr_vision_interfaces/srv/ClipLearnFace_Response";
}

template<>
struct has_fixed_size<lasr_vision_interfaces::srv::ClipLearnFace_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<lasr_vision_interfaces::srv::ClipLearnFace_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<lasr_vision_interfaces::srv::ClipLearnFace_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<lasr_vision_interfaces::srv::ClipLearnFace>()
{
  return "lasr_vision_interfaces::srv::ClipLearnFace";
}

template<>
inline const char * name<lasr_vision_interfaces::srv::ClipLearnFace>()
{
  return "lasr_vision_interfaces/srv/ClipLearnFace";
}

template<>
struct has_fixed_size<lasr_vision_interfaces::srv::ClipLearnFace>
  : std::integral_constant<
    bool,
    has_fixed_size<lasr_vision_interfaces::srv::ClipLearnFace_Request>::value &&
    has_fixed_size<lasr_vision_interfaces::srv::ClipLearnFace_Response>::value
  >
{
};

template<>
struct has_bounded_size<lasr_vision_interfaces::srv::ClipLearnFace>
  : std::integral_constant<
    bool,
    has_bounded_size<lasr_vision_interfaces::srv::ClipLearnFace_Request>::value &&
    has_bounded_size<lasr_vision_interfaces::srv::ClipLearnFace_Response>::value
  >
{
};

template<>
struct is_service<lasr_vision_interfaces::srv::ClipLearnFace>
  : std::true_type
{
};

template<>
struct is_service_request<lasr_vision_interfaces::srv::ClipLearnFace_Request>
  : std::true_type
{
};

template<>
struct is_service_response<lasr_vision_interfaces::srv::ClipLearnFace_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // LASR_VISION_INTERFACES__SRV__DETAIL__CLIP_LEARN_FACE__TRAITS_HPP_
