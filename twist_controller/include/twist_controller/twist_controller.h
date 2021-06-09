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

#include <controller_interface/controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TwistStamped.h>
#include <realtime_tools/realtime_buffer.h>

#include <cartesian_interface/cartesian_command_interface.h>

#include <twist_controller/TwistControllerConfig.h>

namespace ros_controllers_cartesian
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
  void reconfigureCallback(const twist_controller::TwistControllerConfig& config, uint32_t level);
  double gain_;
  std::shared_ptr<dynamic_reconfigure::Server<twist_controller::TwistControllerConfig>> server_;
};

}  // namespace ros_controllers_cartesian
