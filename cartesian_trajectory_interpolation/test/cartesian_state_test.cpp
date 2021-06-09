// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2021 FZI Forschungszentrum Informatik
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

using namespace ros_controllers_cartesian;

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
  // Fill some fields with arbitrary values.
  // Note that jerk, posture and time_from_start do not have a representation
  // in CartesianState and are therefore initialized to 0 (by default) in order to make their string representations to
  // be expected the same..

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
  CartesianState a;
  CartesianState b;
  b.p[0] = 1;
  b.v[0] = 1;
  b.w[0] = 1;
  b.v_dot[0] = 1;
  b.w_dot[0] = 1;

  // Subtraction from zero yields the negative argument
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
