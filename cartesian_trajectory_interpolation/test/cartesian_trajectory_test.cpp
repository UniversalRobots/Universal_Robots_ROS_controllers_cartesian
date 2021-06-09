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

using namespace ros_controllers_cartesian;

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
  p2.pose.orientation.x = 1;  // 180 degrees around x
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
