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

#include <cartesian_interface/cartesian_state_handle.h>

using namespace cartesian_ros_control;

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
