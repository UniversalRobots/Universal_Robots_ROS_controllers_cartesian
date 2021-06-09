// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2020 FZI Forschungszentrum Informatik
// Created on behalf of Universal Robots A/S
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner mauch@fzi.de
 * \date    2020-07-02
 *
 */
//----------------------------------------------------------------------

#pragma once

#include <cartesian_interface/cartesian_state_handle.h>

namespace ros_controllers_cartesian
{
/**
 * @brief A handle for setting pose commands
 *
 * Cartesian ROS-controllers can use this handle to write their control signals
 * to the according PoseCommandInterface.
 */
class PoseCommandHandle : public CartesianStateHandle
{
public:
  PoseCommandHandle() = default;
  PoseCommandHandle(const CartesianStateHandle& state_handle, geometry_msgs::Pose* cmd)
    : CartesianStateHandle(state_handle), cmd_(cmd)
  {
    if (!cmd)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create pose command handle for frame '" +
                                                           state_handle.getName() + "'. Command data pointer is null.");
    }
  }
  virtual ~PoseCommandHandle() = default;

  void setCommand(const geometry_msgs::Pose& pose)
  {
    assert(cmd_);
    *cmd_ = pose;
  }

  geometry_msgs::Pose getCommand() const
  {
    assert(cmd_);
    return *cmd_;
  }
  const geometry_msgs::Pose* getCommandPtr() const
  {
    assert(cmd_);
    return cmd_;
  }

private:
  geometry_msgs::Pose* cmd_ = { nullptr };
};

/**
 * @brief A handle for setting twist commands
 *
 * Cartesian ROS-controllers can use this handle to write their control signals
 * to the according TwistCommandInterface.
 */
class TwistCommandHandle : public CartesianStateHandle
{
public:
  TwistCommandHandle() = default;
  TwistCommandHandle(const CartesianStateHandle& state_handle, geometry_msgs::Twist* cmd)
    : CartesianStateHandle(state_handle), cmd_(cmd)
  {
    if (!cmd)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create twist command handle for frame '" +
                                                           state_handle.getName() + "'. Command data pointer is null.");
    }
  }
  virtual ~TwistCommandHandle() = default;

  void setCommand(const geometry_msgs::Twist& twist)
  {
    assert(cmd_);
    *cmd_ = twist;
  }

  geometry_msgs::Twist getCommand() const
  {
    assert(cmd_);
    return *cmd_;
  }
  const geometry_msgs::Twist* getCommandPtr() const
  {
    assert(cmd_);
    return cmd_;
  }

private:
  geometry_msgs::Twist* cmd_ = { nullptr };
};

/**
 * @brief A Cartesian command interface for poses
 *
 * Use an instance of this class to provide Cartesian ROS-controllers with
 * mechanisms to set poses as commands in the hardware_interface::RobotHW
 * abstraction.
 */
class PoseCommandInterface
  : public hardware_interface::HardwareResourceManager<PoseCommandHandle, hardware_interface::ClaimResources>
{
};

/**
 * @brief A Cartesian command interface for twists
 *
 * Use an instance of this class to provide Cartesian ROS-controllers with
 * mechanisms to set twists as commands in the hardware_interface::RobotHW
 * abstraction.
 */
class TwistCommandInterface
  : public hardware_interface::HardwareResourceManager<TwistCommandHandle, hardware_interface::ClaimResources>
{
};
}  // namespace ros_controllers_cartesian
