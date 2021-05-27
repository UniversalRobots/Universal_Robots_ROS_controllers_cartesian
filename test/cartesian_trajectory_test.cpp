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
/*!\file    cartesian_trajectory_test.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2021/05/25
 *
 */
//-----------------------------------------------------------------------------


#include <gtest/gtest.h>

#include <cartesian_trajectory_interpolation/cartesian_trajectory.h>
#include <cartesian_trajectory_interpolation/cartesian_state.h>
#include <cartesian_control_msgs/CartesianTrajectory.h>
#include "Eigen/src/Core/Matrix.h"
#include "cartesian_control_msgs/CartesianTrajectoryPoint.h"
#include "ros/duration.h"
#include <ros/ros.h>
#include <ostream>

using namespace cartesian_ros_control;

TEST(TestCartesianTrajectory, SamplingFromEmptyTrajectoryGivesEmptyState)
{
  CartesianState a;
  CartesianState b;

  CartesianTrajectory empty_traj;
  empty_traj.sample(-0.5, a);

  std::stringstream a_str;
  std::stringstream b_str;
  a_str << a;
  b_str << b;

  EXPECT_EQ(b_str.str(), a_str.str());
}

TEST(TestCartesianTrajectory, DefaultQuaternionsGiveValidInterpolation)
{
  auto p1 = cartesian_control_msgs::CartesianTrajectoryPoint();
  p1.pose.position.x = 0.0;
  p1.pose.position.y = 0.0;
  p1.pose.position.z = 0.0;
  p1.time_from_start = ros::Duration(0.0);

  auto p2 = cartesian_control_msgs::CartesianTrajectoryPoint();
  p2.pose.position.x = 1.0;
  p2.pose.position.y = 1.0;
  p2.pose.position.z = 1.0;
  p2.time_from_start = ros::Duration(5.0);

  cartesian_control_msgs::CartesianTrajectory traj;
  traj.points.push_back(p1);
  traj.points.push_back(p2);

  auto c_traj = CartesianTrajectory(traj);
  CartesianState c;
  c_traj.sample(2.5, c);

  std::stringstream result;
  result << c;

  // Check for nan values as a frequent result for invalid, all-zero quaternion operations.
  EXPECT_EQ(std::string::npos, result.str().find("nan"));
}

TEST(TestCartesianTrajectory, NonIncreasingWaypointsInTimeFail)
{
  auto p1 = cartesian_control_msgs::CartesianTrajectoryPoint();
  auto p2 = cartesian_control_msgs::CartesianTrajectoryPoint();
  p1.time_from_start = ros::Duration(1.0);
  p2.time_from_start = ros::Duration(0.9);

  cartesian_control_msgs::CartesianTrajectory init;
  init.points.push_back(p1);
  init.points.push_back(p2);

  CartesianTrajectory c_traj;
  EXPECT_FALSE(c_traj.init(init));
}

TEST(TestCartesianTrajectory, InterpolationGivesPlausibleResults)
{
  auto p1 = cartesian_control_msgs::CartesianTrajectoryPoint();
  p1.pose.position.x = 0.0;
  p1.pose.position.y = 0.0;
  p1.pose.position.z = 0.0;
  p1.pose.orientation.x = 0;
  p1.pose.orientation.y = 0;
  p1.pose.orientation.z = 0;
  p1.pose.orientation.w = 1;
  p1.time_from_start = ros::Duration(0.0);

  auto p2 = cartesian_control_msgs::CartesianTrajectoryPoint();
  p2.pose.position.x = 1.1;
  p2.pose.position.y = 2.2;
  p2.pose.position.z = 3.3;
  p2.pose.orientation.x = 1; // 180 degrees around x
  p2.pose.orientation.y = 0;
  p2.pose.orientation.z = 0;
  p2.pose.orientation.w = 0;
  p2.time_from_start = ros::Duration(10.0);

  cartesian_control_msgs::CartesianTrajectory traj;
  traj.points.push_back(p1);
  traj.points.push_back(p2);

  auto c_traj = CartesianTrajectory(traj);
  CartesianState c;
  c_traj.sample(5, c);

  constexpr double pi = 3.14159265358979323846;

  // For the linear case, half the time should give half the values.
  EXPECT_DOUBLE_EQ(p2.pose.position.x / 2.0, c.p.x());
  EXPECT_DOUBLE_EQ(p2.pose.position.y / 2.0, c.p.y());
  EXPECT_DOUBLE_EQ(p2.pose.position.z / 2.0, c.p.z());
  EXPECT_DOUBLE_EQ(pi / 2.0, c.rot().norm());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
