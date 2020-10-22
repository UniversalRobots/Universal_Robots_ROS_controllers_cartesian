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

#include <gtest/gtest.h>

#include <cartesian_interface/cartesian_command_interface.h>
#include <cartesian_interface/cartesian_state_handle.h>

using namespace cartesian_ros_control;

class CartesianCommandInterfaceTest : public ::testing::Test
{
protected:
  std::string reference_frame = "base";
  std::string controlled_frame = "tool0";
  geometry_msgs::Pose pose_buffer;
  geometry_msgs::Twist twist_buffer;
  geometry_msgs::Accel accel_buffer;
  geometry_msgs::Accel jerk_buffer;
  CartesianStateHandle state_handle{ reference_frame, controlled_frame, &pose_buffer,
                                     &twist_buffer,   &accel_buffer,    &jerk_buffer };
  geometry_msgs::Pose pose_cmd_buffer;
  geometry_msgs::Twist twist_cmd_buffer;
  geometry_msgs::Accel accel_cmd_buffer;
  geometry_msgs::Accel jerk_cmd_buffer;
};

TEST_F(CartesianCommandInterfaceTest, TestPoseHandleConstructor)
{
  EXPECT_NO_THROW(PoseCommandHandle obj(state_handle, &pose_cmd_buffer));
  EXPECT_THROW(PoseCommandHandle obj(state_handle, nullptr), hardware_interface::HardwareInterfaceException);
}

TEST_F(CartesianCommandInterfaceTest, TestPoseHandleDataHandling)
{
  PoseCommandHandle cmd_handle(state_handle, &pose_cmd_buffer);

  PoseCommandInterface iface;
  iface.registerHandle(cmd_handle);

  EXPECT_NO_THROW(iface.getHandle(controlled_frame));

  cmd_handle = iface.getHandle(controlled_frame);

  EXPECT_EQ(controlled_frame, cmd_handle.getName());
  geometry_msgs::Pose new_cmd;
  new_cmd.position.x = 1.0;
  new_cmd.position.y = 2.0;
  new_cmd.position.z = 3.0;
  new_cmd.orientation.x = 0.5;
  new_cmd.orientation.y = 0.5;
  new_cmd.orientation.z = 0.0;
  new_cmd.orientation.w = 0.0;
  cmd_handle.setPose(new_cmd);
  EXPECT_DOUBLE_EQ(new_cmd.position.x, cmd_handle.getPose().position.x);
  EXPECT_DOUBLE_EQ(new_cmd.position.y, cmd_handle.getPose().position.y);
  EXPECT_DOUBLE_EQ(new_cmd.position.z, cmd_handle.getPose().position.z);
  EXPECT_DOUBLE_EQ(new_cmd.orientation.x, cmd_handle.getPose().orientation.x);
  EXPECT_DOUBLE_EQ(new_cmd.orientation.y, cmd_handle.getPose().orientation.y);
  EXPECT_DOUBLE_EQ(new_cmd.orientation.z, cmd_handle.getPose().orientation.z);
  EXPECT_DOUBLE_EQ(new_cmd.orientation.w, cmd_handle.getPose().orientation.w);
}

TEST_F(CartesianCommandInterfaceTest, TestTwistHandleConstructor)
{
  EXPECT_NO_THROW(TwistCommandHandle obj(state_handle, &twist_cmd_buffer));
  EXPECT_THROW(TwistCommandHandle obj(state_handle, nullptr), hardware_interface::HardwareInterfaceException);
}

TEST_F(CartesianCommandInterfaceTest, TestTwistHandleDataHandling)
{
  TwistCommandHandle cmd_handle(state_handle, &twist_cmd_buffer);

  TwistCommandInterface iface;
  iface.registerHandle(cmd_handle);

  EXPECT_NO_THROW(iface.getHandle(controlled_frame));

  cmd_handle = iface.getHandle(controlled_frame);

  EXPECT_EQ(controlled_frame, cmd_handle.getName());
  geometry_msgs::Twist new_cmd;
  new_cmd.linear.x = 1.0;
  new_cmd.linear.y = 2.0;
  new_cmd.linear.z = 3.0;
  new_cmd.angular.x = 0.5;
  new_cmd.angular.y = 0.5;
  new_cmd.angular.z = 0.0;
  cmd_handle.setTwist(new_cmd);
  EXPECT_DOUBLE_EQ(new_cmd.linear.x, cmd_handle.getTwist().linear.x);
  EXPECT_DOUBLE_EQ(new_cmd.linear.y, cmd_handle.getTwist().linear.y);
  EXPECT_DOUBLE_EQ(new_cmd.linear.z, cmd_handle.getTwist().linear.z);
  EXPECT_DOUBLE_EQ(new_cmd.angular.x, cmd_handle.getTwist().angular.x);
  EXPECT_DOUBLE_EQ(new_cmd.angular.y, cmd_handle.getTwist().angular.y);
  EXPECT_DOUBLE_EQ(new_cmd.angular.z, cmd_handle.getTwist().angular.z);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
