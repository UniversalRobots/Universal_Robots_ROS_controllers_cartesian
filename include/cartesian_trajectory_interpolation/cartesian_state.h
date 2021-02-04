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

#pragma once

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/AngleAxis.h"
#include "Eigen/src/Geometry/Quaternion.h"

#include <cartesian_control_msgs/CartesianTrajectoryPoint.h>

namespace cartesian_ros_control
{

  /**
   * @brief Cartesian state with pose, velocity and acceleration
   *
   * All quantities are assumed to be given in one common reference frame.
   * This frame is also the reference for the pose defined by \ref p and \ref q.
   *
   */
  struct CartesianState
  {
    CartesianState() = default;

    /**
     * @brief Convenience constructor for ROS messages
     *
     * @param point The desired state
     */
    CartesianState(const cartesian_control_msgs::CartesianTrajectoryPoint& point);

    /**
     * @brief Difference operator between states
     *
     * This is the element-wise difference for all vector quantities and the
     * difference of rotation for the quaternion.
     *
     * @param other State to subtract
     *
     * @return The element-wise difference of the two.
     */
    CartesianState operator-(const CartesianState& other) const;

    /**
     * @brief Convenience method for conversion
     *
     * @param time_from_start Time from start in seconds
     *
     * @return Cartesian State in trajectory waypoint representation
     */
    cartesian_control_msgs::CartesianTrajectoryPoint toMsg(int time_from_start = 0) const;

    /**
     * @brief Get Euler-Rodrigues vector from orientation
     *
     * @return The orientation in axis-angle notation
     */
    Eigen::Vector3d rot() const {Eigen::AngleAxisd a(q); return a.axis() * a.angle();};

    // Pose
    Eigen::Vector3d p; ///< position
    Eigen::Quaterniond q; ///< rotation

    // Spatial velocity in body (waypoint) coordinates
    Eigen::Vector3d v; ///< linear velocity, \f$ v \f$
    Eigen::Vector3d w; ///< angular velocity, \f$ \omega \f$

    // Spatial acceleration in body (waypoint) coordinates
    Eigen::Vector3d v_dot; ///< linear acceleration, \f$ \dot{v} \f$
    Eigen::Vector3d w_dot; ///< angular acceleration, \f$ \dot{\omega} \f$

  };

}
