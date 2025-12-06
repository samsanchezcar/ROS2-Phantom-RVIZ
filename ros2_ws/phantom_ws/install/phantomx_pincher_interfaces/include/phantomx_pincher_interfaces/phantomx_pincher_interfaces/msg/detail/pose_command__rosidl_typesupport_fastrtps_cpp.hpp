// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from phantomx_pincher_interfaces:msg/PoseCommand.idl
// generated code does not contain a copyright notice

#ifndef PHANTOMX_PINCHER_INTERFACES__MSG__DETAIL__POSE_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define PHANTOMX_PINCHER_INTERFACES__MSG__DETAIL__POSE_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "phantomx_pincher_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "phantomx_pincher_interfaces/msg/detail/pose_command__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace phantomx_pincher_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_phantomx_pincher_interfaces
cdr_serialize(
  const phantomx_pincher_interfaces::msg::PoseCommand & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_phantomx_pincher_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  phantomx_pincher_interfaces::msg::PoseCommand & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_phantomx_pincher_interfaces
get_serialized_size(
  const phantomx_pincher_interfaces::msg::PoseCommand & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_phantomx_pincher_interfaces
max_serialized_size_PoseCommand(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace phantomx_pincher_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_phantomx_pincher_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, phantomx_pincher_interfaces, msg, PoseCommand)();

#ifdef __cplusplus
}
#endif

#endif  // PHANTOMX_PINCHER_INTERFACES__MSG__DETAIL__POSE_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
