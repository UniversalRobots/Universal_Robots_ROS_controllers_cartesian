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
/*!\file    cartesian_state.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2021/01/25
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_trajectory_interpolation/cartesian_state.h>
#include <tf2_eigen/tf2_eigen.h>
#include "Eigen/src/Core/Matrix.h"
#include "geometry_msgs/Vector3.h"

namespace my_tf2
{
geometry_msgs::Vector3 toMsg(const Eigen::Vector3d& in)
{
  geometry_msgs::Vector3 msg;
  msg.x = in.x();
  msg.y = in.y();
  msg.z = in.z();
  return msg;
};
}  // namespace my_tf2

namespace ros_controllers_cartesian
{
CartesianState::CartesianState()
{
  p = Eigen::Vector3d::Zero();
  q.x() = 0;
  q.y() = 0;
  q.z() = 0;
  q.w() = 1;

  v = Eigen::Vector3d::Zero();
  v_dot = Eigen::Vector3d::Zero();

  w = Eigen::Vector3d::Zero();
  w_dot = Eigen::Vector3d::Zero();
}

CartesianState::CartesianState(const cartesian_control_msgs::CartesianTrajectoryPoint& point)
{
  // Pose
  tf2::fromMsg(point.pose.position, p);
  tf2::fromMsg(point.pose.orientation, q);
  if (q.coeffs() == Eigen::Vector4d::Zero())
  {
    q.w() = 1;
  }
  q.normalize();

  // Velocity
  tf2::fromMsg(point.twist.linear, v);
  tf2::fromMsg(point.twist.angular, w);

  // Acceleration
  tf2::fromMsg(point.acceleration.linear, v_dot);
  tf2::fromMsg(point.acceleration.angular, w_dot);
}

CartesianState CartesianState::operator-(const CartesianState& other) const
{
  CartesianState result;
  result.p = p - other.p;
  result.q = q * other.q.inverse();
  result.v = v - other.v;
  result.w = w - other.w;
  result.v_dot = v_dot - other.v_dot;
  result.w_dot = w_dot - other.w_dot;

  return result;
}

cartesian_control_msgs::CartesianTrajectoryPoint CartesianState::toMsg(int time_from_start) const
{
  cartesian_control_msgs::CartesianTrajectoryPoint point;

  // Pose
  point.pose.position = tf2::toMsg(p);
  point.pose.orientation = tf2::toMsg(q);

  // Velocity
  point.twist.linear = my_tf2::toMsg(v);
  point.twist.angular = my_tf2::toMsg(w);

  // Acceleration
  point.acceleration.linear = my_tf2::toMsg(v_dot);
  point.acceleration.angular = my_tf2::toMsg(w_dot);

  return point;
}

std::ostream& operator<<(std::ostream& out, const CartesianState& state)
{
  out << "p:\n" << state.p << '\n';
  out << "q:\n" << state.q.coeffs() << '\n';
  out << "v:\n" << state.v << '\n';
  out << "w:\n" << state.w << '\n';
  out << "v_dot:\n" << state.v_dot << '\n';
  out << "w_dot:\n" << state.w_dot;
  return out;
}
}  // namespace ros_controllers_cartesian
