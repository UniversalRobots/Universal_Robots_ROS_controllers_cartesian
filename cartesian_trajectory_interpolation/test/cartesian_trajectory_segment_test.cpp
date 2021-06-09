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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
