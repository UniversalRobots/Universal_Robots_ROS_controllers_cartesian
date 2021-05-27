////////////////////////////////////////////////////////////////////////////////
// Copyright 2021 FZI Research Center for Information Technology
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


//-----------------------------------------------------------------------------
/*!\file    cartesian_state_test.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2021/05/20
 *
 */
//-----------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cartesian_trajectory_interpolation/cartesian_state.h>
#include <Eigen/Dense>
#include <Eigen/src/Geometry/Quaternion.h>
#include <cartesian_control_msgs/CartesianTrajectoryPoint.h>

using namespace cartesian_ros_control;

TEST(TestCartesianState, EmptyStateIsZeroInitialized)  // except for quaternion w
{
  CartesianState state;
  for (size_t i = 0; i < 3; ++i)
  {
    EXPECT_EQ(state.p[i], 0);
    EXPECT_EQ(state.v[i], 0);
    EXPECT_EQ(state.v_dot[i], 0);
    EXPECT_EQ(state.w[i], 0);
    EXPECT_EQ(state.w_dot[i], 0);
  }
  EXPECT_EQ(state.q.x(), 0);
  EXPECT_EQ(state.q.y(), 0);
  EXPECT_EQ(state.q.z(), 0);
  EXPECT_EQ(state.q.w(), 1);
}

TEST(TestCartesianState, EmptyStateHasNormalizedQuaternion)
{
  CartesianState state;
  Eigen::Quaterniond q;
  q.w() = 1;
  q.normalize();
  EXPECT_DOUBLE_EQ(q.norm(), state.q.norm());
}

TEST(TestCartesianState, EmptyStateYieldsZeroRotationVector)
{
  CartesianState state;
  EXPECT_EQ(Eigen::Vector3d::Zero(), state.rot());
}

TEST(TestCartesianState, EmptyStateYieldsNormalizedTrajectoryPoint)
{
  CartesianState state;
  cartesian_control_msgs::CartesianTrajectoryPoint point;
  point.pose.orientation.w = 1;

  // Compact check if both `read` the same.
  std::stringstream state_str;
  std::stringstream point_str;
  state_str << state.toMsg();
  point_str << point;

  EXPECT_EQ(point_str.str(), state_str.str());
}

TEST(TestCartesianState, RosMessageInitializationYieldsNormalizedQuaternions)
{
  cartesian_control_msgs::CartesianTrajectoryPoint init;
  auto c = CartesianState(init);
  EXPECT_DOUBLE_EQ(1.0, c.q.norm());
}

TEST(TestCartesianState, ConversionReturnsInitializingArgument)
{
  // Fill some fields with random values.
  // Note that jerk, posture and time_from_start do not have a representation
  // in CartesianState and are therefore initialized to 0 (by default) in order to make their string representations to be expected the same..

  cartesian_control_msgs::CartesianTrajectoryPoint init;
  init.acceleration.angular.x = 1.2345;
  init.acceleration.linear.y = -173.47;
  init.pose.orientation.w = 1;
  init.pose.position.z = 0.003;
  init.twist.linear.x = 67.594;

  CartesianState state(init);
  std::stringstream init_str;
  std::stringstream state_str;
  init_str << init;
  state_str << state.toMsg();

  EXPECT_EQ(init_str.str(), state_str.str());
}

TEST(CartesianState, RotationDifferenceIsPlausible)
{
  // Subtraction from zero yields the negative argument
  CartesianState a;
  CartesianState b;
  b.p[0] = 1;
  b.v[0] = 1;
  b.w[0] = 1;
  b.v_dot[0] = 1;
  b.w_dot[0] = 1;

  auto diff = a - b;
  EXPECT_DOUBLE_EQ(-b.p[0], diff.p[0]);
  EXPECT_DOUBLE_EQ(-b.v[0], diff.v[0]);
  EXPECT_DOUBLE_EQ(-b.w[0], diff.w[0]);
  EXPECT_DOUBLE_EQ(-b.v_dot[0], diff.v_dot[0]);
  EXPECT_DOUBLE_EQ(-b.w_dot[0], diff.w_dot[0]);

  // Rotations of 180 deg around single axis will have vanishing
  // difference (singularity).
  a.q.x() = 1;
  EXPECT_DOUBLE_EQ(0.0, diff.rot().norm());
  b.q.x() = 1;
  EXPECT_DOUBLE_EQ(0.0, diff.rot().norm());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
