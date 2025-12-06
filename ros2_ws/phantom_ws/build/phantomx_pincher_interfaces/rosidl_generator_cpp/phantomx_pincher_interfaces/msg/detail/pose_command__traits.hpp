// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from phantomx_pincher_interfaces:msg/PoseCommand.idl
// generated code does not contain a copyright notice

#ifndef PHANTOMX_PINCHER_INTERFACES__MSG__DETAIL__POSE_COMMAND__TRAITS_HPP_
#define PHANTOMX_PINCHER_INTERFACES__MSG__DETAIL__POSE_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "phantomx_pincher_interfaces/msg/detail/pose_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace phantomx_pincher_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const PoseCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << ", ";
  }

  // member: roll
  {
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << ", ";
  }

  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: cartesian_path
  {
    out << "cartesian_path: ";
    rosidl_generator_traits::value_to_yaml(msg.cartesian_path, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PoseCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }

  // member: roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << "\n";
  }

  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: cartesian_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cartesian_path: ";
    rosidl_generator_traits::value_to_yaml(msg.cartesian_path, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PoseCommand & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace phantomx_pincher_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use phantomx_pincher_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const phantomx_pincher_interfaces::msg::PoseCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  phantomx_pincher_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use phantomx_pincher_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const phantomx_pincher_interfaces::msg::PoseCommand & msg)
{
  return phantomx_pincher_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<phantomx_pincher_interfaces::msg::PoseCommand>()
{
  return "phantomx_pincher_interfaces::msg::PoseCommand";
}

template<>
inline const char * name<phantomx_pincher_interfaces::msg::PoseCommand>()
{
  return "phantomx_pincher_interfaces/msg/PoseCommand";
}

template<>
struct has_fixed_size<phantomx_pincher_interfaces::msg::PoseCommand>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<phantomx_pincher_interfaces::msg::PoseCommand>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<phantomx_pincher_interfaces::msg::PoseCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PHANTOMX_PINCHER_INTERFACES__MSG__DETAIL__POSE_COMMAND__TRAITS_HPP_
