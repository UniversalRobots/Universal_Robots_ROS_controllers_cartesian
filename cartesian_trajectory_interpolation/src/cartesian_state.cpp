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
}


namespace cartesian_ros_control
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

  std::ostream& operator<<(std::ostream &out, const CartesianState& state)
  {
    out << "p:\n" << state.p << '\n';
    out << "q:\n" << state.q.coeffs() << '\n';
    out << "v:\n" << state.v << '\n';
    out << "w:\n" << state.w << '\n';
    out << "v_dot:\n" << state.v_dot << '\n';
    out << "w_dot:\n" << state.w_dot;
    return out;
  }
}
