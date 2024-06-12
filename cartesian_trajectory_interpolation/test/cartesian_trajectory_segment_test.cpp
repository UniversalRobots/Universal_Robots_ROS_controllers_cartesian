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
/*!\file    cartesian_trajectory_segment_test.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2021/05/25
 *
 */
//-----------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cartesian_trajectory_interpolation/cartesian_trajectory_segment.h>

using namespace ros_controllers_cartesian;

TEST(TestCartesianTrajectorySegment, SamplingBeyondBoundariesIsSafe)
{
  CartesianState start;
  start.p.x() = 1.0;
  start.v.x() = 1.0;
  start.w.x() = 1.0;
  start.v_dot.x() = 1.0;
  start.w_dot.x() = 1.0;
  double start_time = 0;

  CartesianState end;
  end.p.x() = 2.0;
  end.v.x() = 2.0;
  end.w.x() = 2.0;
  end.v_dot.x() = 2.0;
  end.w_dot.x() = 2.0;
  double end_time = 5.0;

  CartesianTrajectorySegment seg{ start_time, start, end_time, end };

  // Sampling beyond the time boundaries should yield the boundary states with
  // zero velocities and zero accelerations.

  CartesianState sample_lower;
  seg.sample(-1.0, sample_lower);

  EXPECT_DOUBLE_EQ(start.p.x(), sample_lower.p.x());
  EXPECT_DOUBLE_EQ(0, sample_lower.v.x());
  EXPECT_DOUBLE_EQ(0, sample_lower.w.x());
  EXPECT_DOUBLE_EQ(0, sample_lower.v_dot.x());
  EXPECT_DOUBLE_EQ(0, sample_lower.w_dot.x());

  CartesianState sample_upper;
  seg.sample(6.0, sample_lower);

  EXPECT_DOUBLE_EQ(end.p.x(), sample_lower.p.x());
  EXPECT_DOUBLE_EQ(0, sample_lower.v.x());
  EXPECT_DOUBLE_EQ(0, sample_lower.w.x());
  EXPECT_DOUBLE_EQ(0, sample_lower.v_dot.x());
  EXPECT_DOUBLE_EQ(0, sample_lower.w_dot.x());
}

TEST(TestCartesianTrajectorySegment, DoubleConversionIsIdentity)
{
  //--------------------------------------------------------------------------------
  // Cartesian -> Spline -> Cartesian
  //--------------------------------------------------------------------------------
  CartesianState c;

  c.v.x() = 1.1;
  c.v.y() = 1.2;
  c.v.z() = 1.3;

  c.w.x() = 2.1;
  c.w.y() = 2.2;
  c.w.z() = 2.3;

  c.v_dot.x() = 3.1;
  c.v_dot.y() = 3.2;
  c.v_dot.z() = 3.3;

  c.w_dot.x() = 4.1;
  c.w_dot.y() = 4.2;
  c.w_dot.z() = 4.3;

  std::stringstream before;
  std::stringstream after;
  before << c;
  after << convert(convert(c));

  // Compact check if both `read` the same.
  EXPECT_EQ(before.str(), after.str());

  // The non-initialized case
  CartesianState d;
  before.clear();
  after.clear();
  before << d;
  after << convert(convert(d));
  EXPECT_EQ(before.str(), after.str());
}

TEST(TestCartesianTrajectorySegment, NansYieldEmptySplineVelocities)
{
  CartesianState c;
  c.p.x() = 1.0;
  c.p.y() = 2.0;
  c.p.z() = 3.0;
  c.q.w() = 4.0;
  c.q.x() = 5.0;
  c.q.y() = 6.0;
  c.q.z() = 7.0;

  c.v.x() = std::nan("0");

  CartesianTrajectorySegment::SplineState s;
  s.position.push_back(1.0);
  s.position.push_back(2.0);
  s.position.push_back(3.0);
  s.position.push_back(4.0);
  s.position.push_back(5.0);
  s.position.push_back(6.0);
  s.position.push_back(7.0);
  std::stringstream expected;
  expected << s;

  s = convert(c);
  std::stringstream actual;
  actual << s;
  EXPECT_EQ(expected.str(), actual.str());
}

TEST(TestCartesianTrajectorySegment, ConvertToSplineState)
{
  CartesianState cartesian_state;
  // Position
  cartesian_state.p.x() = 0.1;
  cartesian_state.p.y() = 0.2;
  cartesian_state.p.z() = 0.3;
  cartesian_state.q.w() = 0.0;
  cartesian_state.q.x() = -0.707107;
  cartesian_state.q.y() = -0.707107;
  cartesian_state.q.z() = 0.0;
  // Velocity
  cartesian_state.v[0] = 0.1;
  cartesian_state.v[1] = 0.1;
  cartesian_state.v[2] = 0.1;
  cartesian_state.w[0] = 0.1;
  cartesian_state.w[1] = 0.1;
  cartesian_state.w[2] = 0.1;
  // Acceleration
  cartesian_state.v_dot[0] = 0.1;
  cartesian_state.v_dot[1] = 0.1;
  cartesian_state.v_dot[2] = 0.1;
  cartesian_state.w_dot[0] = 0.1;
  cartesian_state.w_dot[1] = 0.1;
  cartesian_state.w_dot[2] = 0.1;

  CartesianTrajectorySegment::SplineState expected_spline_state;
  // Position
  expected_spline_state.position.push_back(0.1);
  expected_spline_state.position.push_back(0.2);
  expected_spline_state.position.push_back(0.3);
  expected_spline_state.position.push_back(0.0);
  expected_spline_state.position.push_back(-0.707107);
  expected_spline_state.position.push_back(-0.707107);
  expected_spline_state.position.push_back(0.0);
  // Velocity
  expected_spline_state.velocity.push_back(0.1);
  expected_spline_state.velocity.push_back(0.1);
  expected_spline_state.velocity.push_back(-0.0999999);
  expected_spline_state.velocity.push_back(0.0707107);
  expected_spline_state.velocity.push_back(0.0353553);
  expected_spline_state.velocity.push_back(-0.0353553);
  expected_spline_state.velocity.push_back(0.0);
  // Acceleration
  expected_spline_state.acceleration.push_back(0.1);
  expected_spline_state.acceleration.push_back(0.1);
  expected_spline_state.acceleration.push_back(-0.0999999);
  expected_spline_state.acceleration.push_back(0.0707107);
  expected_spline_state.acceleration.push_back(0.0406586);
  expected_spline_state.acceleration.push_back(-0.030052);
  expected_spline_state.acceleration.push_back(0.0);

  std::stringstream expected;
  expected << expected_spline_state;

  CartesianTrajectorySegment::SplineState actual_spline_state = convert(cartesian_state);
  std::stringstream actual;
  actual << actual_spline_state;

  EXPECT_EQ(expected.str(), actual.str());
}

TEST(TestCartesianTrajectorySegment, ConvertToCartesianState)
{
  CartesianTrajectorySegment::SplineState spline_state;
  // Position
  spline_state.position.push_back(0.1);
  spline_state.position.push_back(0.2);
  spline_state.position.push_back(0.3);
  spline_state.position.push_back(0.0);
  spline_state.position.push_back(-0.707107);
  spline_state.position.push_back(-0.707107);
  spline_state.position.push_back(0.0);
  // Velocity
  spline_state.velocity.push_back(0.1);
  spline_state.velocity.push_back(0.1);
  spline_state.velocity.push_back(0.1);
  spline_state.velocity.push_back(0);
  spline_state.velocity.push_back(0.05);
  spline_state.velocity.push_back(0.05);
  spline_state.velocity.push_back(0.05);
  // Acceleration
  spline_state.acceleration.push_back(0.1);
  spline_state.acceleration.push_back(0.1);
  spline_state.acceleration.push_back(0.1);
  spline_state.acceleration.push_back(-0.0075);
  spline_state.acceleration.push_back(0.05);
  spline_state.acceleration.push_back(0.05);
  spline_state.acceleration.push_back(0.05);

  CartesianState expected_cartesian_state;
  // Position
  expected_cartesian_state.p.x() = 0.1;
  expected_cartesian_state.p.y() = 0.2;
  expected_cartesian_state.p.z() = 0.3;
  expected_cartesian_state.q.w() = 0;
  expected_cartesian_state.q.x() = -0.707107;
  expected_cartesian_state.q.y() = -0.707107;
  expected_cartesian_state.q.z() = 0.0;
  // Velocity
  expected_cartesian_state.v[0] = 0.1;
  expected_cartesian_state.v[1] = 0.1;
  expected_cartesian_state.v[2] = -0.1;
  expected_cartesian_state.w[0] = -0.0707107;
  expected_cartesian_state.w[1] = 0.0707107;
  expected_cartesian_state.w[2] = 0.0;
  // Acceleration
  expected_cartesian_state.v_dot[0] = 0.1;
  expected_cartesian_state.v_dot[1] = 0.1;
  expected_cartesian_state.v_dot[2] = -0.1;
  expected_cartesian_state.w_dot[0] = -0.0913173;
  expected_cartesian_state.w_dot[1] = 0.0701041;
  expected_cartesian_state.w_dot[2] = 0.0;

  std::stringstream expected;
  expected << expected_cartesian_state;

  CartesianState actual_cartesian_state = convert(spline_state);
  std::stringstream actual;
  actual << actual_cartesian_state;

  EXPECT_EQ(expected.str(), actual.str());
}

TEST(TestCartesianTrajectorySegment, interpolationOfOrientation)
{
  // This will test that the orientation is interpolated correctly. The trajectory is based on a sine wave, such that
  // the first and second derivatives are known. The sine wave will produce an angle that is rotated around the z-axis
  // of an "artificial" robot's TCP. This orientation is then rotated to the base of the "artificial" robot. This will
  // make sure that we catch that the interpolation of the orientation is done correctly.

  // Convenience method
  auto compute_cartesian_state = [](auto& cartesian_state, const auto& time) {
    double omega = 0.2;
    double angle = sin(omega * time);

    // Rotation around the "artificial" robot's TCP z-axis
    Eigen::Matrix3d rot_z = Eigen::Matrix3d::Zero();
    rot_z(0, 0) = cos(angle);
    rot_z(0, 1) = -sin(angle);
    rot_z(1, 0) = sin(angle);
    rot_z(1, 1) = cos(angle);
    rot_z(2, 2) = 1;

    // Rotate the z-axis rotation to the base of the "artificial" robot
    Eigen::Matrix3d rot_base = Eigen::Matrix3d::Zero();
    rot_base(0, 1) = 1.0;
    rot_base(1, 0) = 1.0;
    rot_base(2, 2) = -1.0;
    rot_z = rot_base * rot_z;

    // Create cartesian state
    Eigen::Quaterniond q(rot_z);

    cartesian_state.p.x() = 0.0;
    cartesian_state.p.y() = 0.0;
    cartesian_state.p.z() = 0.0;

    cartesian_state.q.w() = q.w();
    cartesian_state.q.x() = q.x();
    cartesian_state.q.y() = q.y();
    cartesian_state.q.z() = q.z();

    cartesian_state.v[0] = 0.0;
    cartesian_state.v[1] = 0.0;
    cartesian_state.v[2] = 0.0;

    cartesian_state.w[0] = 0.0;
    cartesian_state.w[1] = 0.0;
    cartesian_state.w[2] = omega * cos(omega * time);
    // Rotate the velocity to the base of the "artificial" robot
    cartesian_state.w = rot_base * cartesian_state.w;

    cartesian_state.v_dot[0] = 0.0;
    cartesian_state.v_dot[0] = 0.0;
    cartesian_state.v_dot[0] = 0.0;

    cartesian_state.w_dot[0] = 0.0;
    cartesian_state.w_dot[1] = 0.0;
    cartesian_state.w_dot[2] = (-omega * omega) * sin(omega * time);
    // Rotate the acceleration to the base of the "artificial" robot
    cartesian_state.w_dot = rot_base * cartesian_state.w_dot;
  };

  double start_time = 0.0;
  double end_time = 1.0;

  // Setup states
  CartesianState start_state;
  compute_cartesian_state(start_state, start_time);

  CartesianState end_state;
  compute_cartesian_state(end_state, end_time);

  // Create segment
  CartesianTrajectorySegment segment{ start_time, start_state, end_time, end_state };

  CartesianState sampled_state;
  segment.sample(0.3, sampled_state);

  CartesianState expected_state;
  compute_cartesian_state(expected_state, 0.3);

  // Orientation
  double eps = 1e-3;
  EXPECT_NEAR(expected_state.q.x(), sampled_state.q.x(), eps);
  EXPECT_NEAR(expected_state.q.y(), sampled_state.q.y(), eps);
  EXPECT_NEAR(expected_state.q.z(), sampled_state.q.z(), eps);
  EXPECT_NEAR(expected_state.q.w(), sampled_state.q.w(), eps);

  // Velocity
  EXPECT_NEAR(expected_state.w[0], sampled_state.w[0], eps);
  EXPECT_NEAR(expected_state.w[1], sampled_state.w[1], eps);
  EXPECT_NEAR(expected_state.w[2], sampled_state.w[2], eps);

  // Acceleration
  EXPECT_NEAR(expected_state.w_dot[0], sampled_state.w_dot[0], eps);
  EXPECT_NEAR(expected_state.w_dot[1], sampled_state.w_dot[1], eps);
  EXPECT_NEAR(expected_state.w_dot[2], sampled_state.w_dot[2], eps);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
