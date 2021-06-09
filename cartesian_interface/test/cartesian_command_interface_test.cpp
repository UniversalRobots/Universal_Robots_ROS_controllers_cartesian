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

#include <gtest/gtest.h>

#include <cartesian_interface/cartesian_command_interface.h>
#include <cartesian_interface/cartesian_state_handle.h>

using namespace ros_controllers_cartesian;

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
  cmd_handle.setCommand(new_cmd);
  EXPECT_DOUBLE_EQ(new_cmd.position.x, cmd_handle.getCommand().position.x);
  EXPECT_DOUBLE_EQ(new_cmd.position.y, cmd_handle.getCommand().position.y);
  EXPECT_DOUBLE_EQ(new_cmd.position.z, cmd_handle.getCommand().position.z);
  EXPECT_DOUBLE_EQ(new_cmd.orientation.x, cmd_handle.getCommand().orientation.x);
  EXPECT_DOUBLE_EQ(new_cmd.orientation.y, cmd_handle.getCommand().orientation.y);
  EXPECT_DOUBLE_EQ(new_cmd.orientation.z, cmd_handle.getCommand().orientation.z);
  EXPECT_DOUBLE_EQ(new_cmd.orientation.w, cmd_handle.getCommand().orientation.w);
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
  cmd_handle.setCommand(new_cmd);
  EXPECT_DOUBLE_EQ(new_cmd.linear.x, cmd_handle.getCommand().linear.x);
  EXPECT_DOUBLE_EQ(new_cmd.linear.y, cmd_handle.getCommand().linear.y);
  EXPECT_DOUBLE_EQ(new_cmd.linear.z, cmd_handle.getCommand().linear.z);
  EXPECT_DOUBLE_EQ(new_cmd.angular.x, cmd_handle.getCommand().angular.x);
  EXPECT_DOUBLE_EQ(new_cmd.angular.y, cmd_handle.getCommand().angular.y);
  EXPECT_DOUBLE_EQ(new_cmd.angular.z, cmd_handle.getCommand().angular.z);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
