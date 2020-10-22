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
 * \author   mauch@fzi.de
 * \date    2020-07-01
 *
 */
//----------------------------------------------------------------------

#pragma once

#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace cartesian_ros_control
{

/**
 * @brief A state handle for Cartesian hardware interfaces
 *
 * Cartesian ROS-controllers can use this handle to read the current Cartesian
 * state from the Cartesian HW-interface and use that in their control loops.
 * The functionality is analog to how the joint-based handles work:
 * Implementers of the hardware_interface::RobotHW class provide a set of
 * buffers to this handle upon instantiation and register this handle with an
 * instance of the according CartesianStateInterface.
 *
 */
class CartesianStateHandle
{
public:
  CartesianStateHandle() = default;
  CartesianStateHandle(const std::string& ref_frame_id, const std::string& frame_id, const geometry_msgs::Pose* pose,
                       const geometry_msgs::Twist* twist, const geometry_msgs::Accel* accel,
                       const geometry_msgs::Accel* jerk)
    : frame_id_(frame_id), ref_frame_id_(ref_frame_id), pose_(pose), twist_(twist), accel_(accel), jerk_(jerk)
  {
    if (!pose)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create Cartesian handle for frame '" + frame_id_ +
                                                           "'. Pose data pointer is null.");
    }
    if (!twist)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create Cartesian handle for frame '" + frame_id_ +
                                                           "'. Twist data pointer is null.");
    }
    if (!accel)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create Cartesian handle for frame '" + frame_id_ +
                                                           "'. Accel data pointer is null.");
    }
    if (!jerk)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create Cartesian handle for frame '" + frame_id_ +
                                                           "'. Jerk data pointer is null.");
    }
  }
  virtual ~CartesianStateHandle() = default;

  std::string getName() const
  {
    return frame_id_;
  }
  geometry_msgs::Pose getPose() const
  {
    assert(pose_);
    return *pose_;
  }
  geometry_msgs::Twist getTwist() const
  {
    assert(twist_);
    return *twist_;
  }
  geometry_msgs::Accel getAccel() const
  {
    assert(accel_);
    return *accel_;
  }
  geometry_msgs::Accel getJerk() const
  {
    assert(jerk_);
    return *jerk_;
  }

private:
  std::string frame_id_;
  std::string ref_frame_id_;
  const geometry_msgs::Pose* pose_;
  const geometry_msgs::Twist* twist_;
  const geometry_msgs::Accel* accel_;
  const geometry_msgs::Accel* jerk_;
};

/**
 * @brief A Cartesian state interface for hardware_interface::RobotHW abstractions
 *
 * This interface can be passed to Cartesian ROS-controllers as hardware type during initialization.
 * The controllers then obtain read access to the underlying buffers via the \a CartesianStateHandle.
 */
class CartesianStateInterface : public hardware_interface::HardwareResourceManager<CartesianStateHandle>
{
};
}  // namespace cartesian_ros_control
