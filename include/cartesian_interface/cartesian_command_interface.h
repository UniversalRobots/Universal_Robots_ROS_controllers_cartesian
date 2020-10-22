////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

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

namespace cartesian_ros_control
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

  void setPose(const geometry_msgs::Pose& pose)
  {
    assert(cmd_);
    *cmd_ = pose;
  }

  geometry_msgs::Pose getPose() const
  {
    assert(cmd_);
    return *cmd_;
  }
  const geometry_msgs::Pose* getPosePtr() const
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

  void setTwist(const geometry_msgs::Twist& twist)
  {
    assert(cmd_);
    *cmd_ = twist;
  }

  geometry_msgs::Twist getTwist() const
  {
    assert(cmd_);
    return *cmd_;
  }
  const geometry_msgs::Twist* getTwistPtr() const
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
}  // namespace cartesian_ros_control
