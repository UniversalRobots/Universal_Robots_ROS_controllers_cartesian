////////////////////////////////////////////////////////////////////////////////
// Copyright 2020 FZI Research Center for Information Technology
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

#include <controller_interface/controller.h>
#include <geometry_msgs/TwistStamped.h>
#include <realtime_tools/realtime_buffer.h>

#include <cartesian_interface/cartesian_command_interface.h>

namespace cartesian_ros_control
{

/**
 * @brief A Cartesian ROS-controller for commanding target twists to a robot
 *
 * This controller makes use of a TwistCommandInterface to set a user specified
 * twist message as reference for robot control.
 * The according hardware_interface::RobotHW can send these commands
 * directly to the robot driver in its write() function.
 */
class TwistController : public controller_interface::Controller<TwistCommandInterface>
{
public:
  TwistController() = default;
  virtual ~TwistController() = default;

  virtual bool init(TwistCommandInterface* hw, ros::NodeHandle& n) override;

  virtual void starting(const ros::Time& time) override;

  virtual void update(const ros::Time& /*time*/, const ros::Duration& /*period*/) override
  {
    handle_.setCommand(*command_buffer_.readFromRT());
  }

  TwistCommandHandle handle_;
  realtime_tools::RealtimeBuffer<geometry_msgs::Twist> command_buffer_;

private:
  ros::Subscriber twist_sub_;
  void twistCallback(const geometry_msgs::TwistConstPtr& msg);
  double gain_ = { 0.1 };
};

}  // namespace cartesian_ros_control
