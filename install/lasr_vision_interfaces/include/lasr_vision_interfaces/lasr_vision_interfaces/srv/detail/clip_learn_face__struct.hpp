// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from lasr_vision_interfaces:srv/ClipLearnFace.idl
// generated code does not contain a copyright notice

#ifndef LASR_VISION_INTERFACES__SRV__DETAIL__CLIP_LEARN_FACE__STRUCT_HPP_
#define LASR_VISION_INTERFACES__SRV__DETAIL__CLIP_LEARN_FACE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'raw_imgs'
#include "sensor_msgs/msg/detail/image__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__lasr_vision_interfaces__srv__ClipLearnFace_Request __attribute__((deprecated))
#else
# define DEPRECATED__lasr_vision_interfaces__srv__ClipLearnFace_Request __declspec(deprecated)
#endif

namespace lasr_vision_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ClipLearnFace_Request_
{
  using Type = ClipLearnFace_Request_<ContainerAllocator>;

  explicit ClipLearnFace_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
    }
  }

  explicit ClipLearnFace_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
    }
  }

  // field types and members
  using _raw_imgs_type =
    std::vector<sensor_msgs::msg::Image_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sensor_msgs::msg::Image_<ContainerAllocator>>>;
  _raw_imgs_type raw_imgs;
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;

  // setters for named parameter idiom
  Type & set__raw_imgs(
    const std::vector<sensor_msgs::msg::Image_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sensor_msgs::msg::Image_<ContainerAllocator>>> & _arg)
  {
    this->raw_imgs = _arg;
    return *this;
  }
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    lasr_vision_interfaces::srv::ClipLearnFace_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const lasr_vision_interfaces::srv::ClipLearnFace_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lasr_vision_interfaces::srv::ClipLearnFace_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lasr_vision_interfaces::srv::ClipLearnFace_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lasr_vision_interfaces::srv::ClipLearnFace_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lasr_vision_interfaces::srv::ClipLearnFace_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lasr_vision_interfaces::srv::ClipLearnFace_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lasr_vision_interfaces::srv::ClipLearnFace_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lasr_vision_interfaces::srv::ClipLearnFace_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lasr_vision_interfaces::srv::ClipLearnFace_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lasr_vision_interfaces__srv__ClipLearnFace_Request
    std::shared_ptr<lasr_vision_interfaces::srv::ClipLearnFace_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lasr_vision_interfaces__srv__ClipLearnFace_Request
    std::shared_ptr<lasr_vision_interfaces::srv::ClipLearnFace_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ClipLearnFace_Request_ & other) const
  {
    if (this->raw_imgs != other.raw_imgs) {
      return false;
    }
    if (this->name != other.name) {
      return false;
    }
    return true;
  }
  bool operator!=(const ClipLearnFace_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ClipLearnFace_Request_

// alias to use template instance with default allocator
using ClipLearnFace_Request =
  lasr_vision_interfaces::srv::ClipLearnFace_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace lasr_vision_interfaces


#ifndef _WIN32
# define DEPRECATED__lasr_vision_interfaces__srv__ClipLearnFace_Response __attribute__((deprecated))
#else
# define DEPRECATED__lasr_vision_interfaces__srv__ClipLearnFace_Response __declspec(deprecated)
#endif

namespace lasr_vision_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ClipLearnFace_Response_
{
  using Type = ClipLearnFace_Response_<ContainerAllocator>;

  explicit ClipLearnFace_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit ClipLearnFace_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    lasr_vision_interfaces::srv::ClipLearnFace_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const lasr_vision_interfaces::srv::ClipLearnFace_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lasr_vision_interfaces::srv::ClipLearnFace_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lasr_vision_interfaces::srv::ClipLearnFace_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lasr_vision_interfaces::srv::ClipLearnFace_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lasr_vision_interfaces::srv::ClipLearnFace_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lasr_vision_interfaces::srv::ClipLearnFace_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lasr_vision_interfaces::srv::ClipLearnFace_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lasr_vision_interfaces::srv::ClipLearnFace_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lasr_vision_interfaces::srv::ClipLearnFace_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lasr_vision_interfaces__srv__ClipLearnFace_Response
    std::shared_ptr<lasr_vision_interfaces::srv::ClipLearnFace_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lasr_vision_interfaces__srv__ClipLearnFace_Response
    std::shared_ptr<lasr_vision_interfaces::srv::ClipLearnFace_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ClipLearnFace_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const ClipLearnFace_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ClipLearnFace_Response_

// alias to use template instance with default allocator
using ClipLearnFace_Response =
  lasr_vision_interfaces::srv::ClipLearnFace_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace lasr_vision_interfaces

namespace lasr_vision_interfaces
{

namespace srv
{

struct ClipLearnFace
{
  using Request = lasr_vision_interfaces::srv::ClipLearnFace_Request;
  using Response = lasr_vision_interfaces::srv::ClipLearnFace_Response;
};

}  // namespace srv

}  // namespace lasr_vision_interfaces

#endif  // LASR_VISION_INTERFACES__SRV__DETAIL__CLIP_LEARN_FACE__STRUCT_HPP_
