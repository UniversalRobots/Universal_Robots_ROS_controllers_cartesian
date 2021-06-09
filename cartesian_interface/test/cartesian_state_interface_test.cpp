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

#include <cartesian_interface/cartesian_state_handle.h>

using namespace ros_controllers_cartesian;

TEST(CartesianStateHandleTest, TestConstructor)
{
  std::string reference_frame = "base";
  std::string controlled_frame = "tool0";
  geometry_msgs::Pose pose_buffer;
  geometry_msgs::Twist twist_buffer;
  geometry_msgs::Accel accel_buffer;
  geometry_msgs::Accel jerk_buffer;

  EXPECT_NO_THROW(CartesianStateHandle obj(reference_frame, controlled_frame, &pose_buffer, &twist_buffer,
                                           &accel_buffer, &jerk_buffer));
  EXPECT_THROW(
      CartesianStateHandle obj(reference_frame, controlled_frame, nullptr, &twist_buffer, &accel_buffer, &jerk_buffer),
      hardware_interface::HardwareInterfaceException);
  EXPECT_THROW(
      CartesianStateHandle obj(reference_frame, controlled_frame, &pose_buffer, nullptr, &accel_buffer, &jerk_buffer),
      hardware_interface::HardwareInterfaceException);
  EXPECT_THROW(
      CartesianStateHandle obj(reference_frame, controlled_frame, &pose_buffer, &twist_buffer, nullptr, &jerk_buffer),
      hardware_interface::HardwareInterfaceException);
  EXPECT_THROW(
      CartesianStateHandle obj(reference_frame, controlled_frame, &pose_buffer, &twist_buffer, &accel_buffer, nullptr),
      hardware_interface::HardwareInterfaceException);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
