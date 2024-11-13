// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lasr_vision_interfaces:srv/ClipLearnFace.idl
// generated code does not contain a copyright notice

#ifndef LASR_VISION_INTERFACES__SRV__DETAIL__CLIP_LEARN_FACE__BUILDER_HPP_
#define LASR_VISION_INTERFACES__SRV__DETAIL__CLIP_LEARN_FACE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "lasr_vision_interfaces/srv/detail/clip_learn_face__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace lasr_vision_interfaces
{

namespace srv
{

namespace builder
{

class Init_ClipLearnFace_Request_name
{
public:
  explicit Init_ClipLearnFace_Request_name(::lasr_vision_interfaces::srv::ClipLearnFace_Request & msg)
  : msg_(msg)
  {}
  ::lasr_vision_interfaces::srv::ClipLearnFace_Request name(::lasr_vision_interfaces::srv::ClipLearnFace_Request::_name_type arg)
  {
    msg_.name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lasr_vision_interfaces::srv::ClipLearnFace_Request msg_;
};

class Init_ClipLearnFace_Request_raw_imgs
{
public:
  Init_ClipLearnFace_Request_raw_imgs()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ClipLearnFace_Request_name raw_imgs(::lasr_vision_interfaces::srv::ClipLearnFace_Request::_raw_imgs_type arg)
  {
    msg_.raw_imgs = std::move(arg);
    return Init_ClipLearnFace_Request_name(msg_);
  }

private:
  ::lasr_vision_interfaces::srv::ClipLearnFace_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::lasr_vision_interfaces::srv::ClipLearnFace_Request>()
{
  return lasr_vision_interfaces::srv::builder::Init_ClipLearnFace_Request_raw_imgs();
}

}  // namespace lasr_vision_interfaces


namespace lasr_vision_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::lasr_vision_interfaces::srv::ClipLearnFace_Response>()
{
  return ::lasr_vision_interfaces::srv::ClipLearnFace_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace lasr_vision_interfaces

#endif  // LASR_VISION_INTERFACES__SRV__DETAIL__CLIP_LEARN_FACE__BUILDER_HPP_
