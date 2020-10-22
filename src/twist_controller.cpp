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

#include <twist_controller/twist_controller.h>
#include <pluginlib/class_list_macros.hpp>

namespace cartesian_ros_control
{
bool TwistController::init(TwistCommandInterface* hw, ros::NodeHandle& n)
{
  std::string frame_id;
  if (!n.getParam("frame_id", frame_id))
  {
    ROS_ERROR_STREAM("Required parameter " << n.resolveName("frame_id") << " not given");
    return false;
  }

  handle_ = hw->getHandle(frame_id);
  twist_sub_ = n.subscribe<geometry_msgs::Twist>("command", 1, &TwistController::twistCallback, this);

  std::vector<std::string> joint_names;
  if (!n.getParam("joints", joint_names))
  {
    ROS_ERROR_STREAM("Failed to read required parameter '" << n.resolveName("joints") << ".");
    return false;
  }

  for (auto& name : joint_names)
  {
    hw->claim(name);
  }

  return true;
}

void TwistController::starting(const ros::Time& time)
{
  geometry_msgs::Twist twist;
  twist.linear.x = 0;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = 0;
  command_buffer_.writeFromNonRT(twist);
}

void TwistController::twistCallback(const geometry_msgs::TwistConstPtr& msg)
{
  geometry_msgs::Twist twist;
  twist.linear.x = gain_ * msg->linear.x;
  twist.linear.y = gain_ * msg->linear.y;
  twist.linear.z = gain_ * msg->linear.z;
  twist.angular.x = gain_ * msg->angular.x;
  twist.angular.y = gain_ * msg->angular.y;
  twist.angular.z = gain_ * msg->angular.z;
  command_buffer_.writeFromNonRT(twist);
}
}  // namespace cartesian_ros_control

PLUGINLIB_EXPORT_CLASS(cartesian_ros_control::TwistController, controller_interface::ControllerBase)
