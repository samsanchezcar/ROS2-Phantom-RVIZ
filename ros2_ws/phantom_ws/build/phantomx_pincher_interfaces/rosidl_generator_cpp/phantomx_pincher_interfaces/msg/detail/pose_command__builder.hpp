// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from phantomx_pincher_interfaces:msg/PoseCommand.idl
// generated code does not contain a copyright notice

#ifndef PHANTOMX_PINCHER_INTERFACES__MSG__DETAIL__POSE_COMMAND__BUILDER_HPP_
#define PHANTOMX_PINCHER_INTERFACES__MSG__DETAIL__POSE_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "phantomx_pincher_interfaces/msg/detail/pose_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace phantomx_pincher_interfaces
{

namespace msg
{

namespace builder
{

class Init_PoseCommand_cartesian_path
{
public:
  explicit Init_PoseCommand_cartesian_path(::phantomx_pincher_interfaces::msg::PoseCommand & msg)
  : msg_(msg)
  {}
  ::phantomx_pincher_interfaces::msg::PoseCommand cartesian_path(::phantomx_pincher_interfaces::msg::PoseCommand::_cartesian_path_type arg)
  {
    msg_.cartesian_path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::phantomx_pincher_interfaces::msg::PoseCommand msg_;
};

class Init_PoseCommand_yaw
{
public:
  explicit Init_PoseCommand_yaw(::phantomx_pincher_interfaces::msg::PoseCommand & msg)
  : msg_(msg)
  {}
  Init_PoseCommand_cartesian_path yaw(::phantomx_pincher_interfaces::msg::PoseCommand::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_PoseCommand_cartesian_path(msg_);
  }

private:
  ::phantomx_pincher_interfaces::msg::PoseCommand msg_;
};

class Init_PoseCommand_pitch
{
public:
  explicit Init_PoseCommand_pitch(::phantomx_pincher_interfaces::msg::PoseCommand & msg)
  : msg_(msg)
  {}
  Init_PoseCommand_yaw pitch(::phantomx_pincher_interfaces::msg::PoseCommand::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_PoseCommand_yaw(msg_);
  }

private:
  ::phantomx_pincher_interfaces::msg::PoseCommand msg_;
};

class Init_PoseCommand_roll
{
public:
  explicit Init_PoseCommand_roll(::phantomx_pincher_interfaces::msg::PoseCommand & msg)
  : msg_(msg)
  {}
  Init_PoseCommand_pitch roll(::phantomx_pincher_interfaces::msg::PoseCommand::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_PoseCommand_pitch(msg_);
  }

private:
  ::phantomx_pincher_interfaces::msg::PoseCommand msg_;
};

class Init_PoseCommand_z
{
public:
  explicit Init_PoseCommand_z(::phantomx_pincher_interfaces::msg::PoseCommand & msg)
  : msg_(msg)
  {}
  Init_PoseCommand_roll z(::phantomx_pincher_interfaces::msg::PoseCommand::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_PoseCommand_roll(msg_);
  }

private:
  ::phantomx_pincher_interfaces::msg::PoseCommand msg_;
};

class Init_PoseCommand_y
{
public:
  explicit Init_PoseCommand_y(::phantomx_pincher_interfaces::msg::PoseCommand & msg)
  : msg_(msg)
  {}
  Init_PoseCommand_z y(::phantomx_pincher_interfaces::msg::PoseCommand::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_PoseCommand_z(msg_);
  }

private:
  ::phantomx_pincher_interfaces::msg::PoseCommand msg_;
};

class Init_PoseCommand_x
{
public:
  Init_PoseCommand_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PoseCommand_y x(::phantomx_pincher_interfaces::msg::PoseCommand::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_PoseCommand_y(msg_);
  }

private:
  ::phantomx_pincher_interfaces::msg::PoseCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::phantomx_pincher_interfaces::msg::PoseCommand>()
{
  return phantomx_pincher_interfaces::msg::builder::Init_PoseCommand_x();
}

}  // namespace phantomx_pincher_interfaces

#endif  // PHANTOMX_PINCHER_INTERFACES__MSG__DETAIL__POSE_COMMAND__BUILDER_HPP_
