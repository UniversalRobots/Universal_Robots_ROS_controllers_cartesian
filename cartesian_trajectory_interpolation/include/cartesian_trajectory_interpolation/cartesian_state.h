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

#pragma once

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/AngleAxis.h"
#include "Eigen/src/Geometry/Quaternion.h"

#include <cartesian_control_msgs/CartesianTrajectoryPoint.h>

namespace ros_controllers_cartesian
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
  /**
   * @brief Initializes all quantities to zero and sets the orientation quaternion to identity
   */
  CartesianState();

  /**
   * @brief Convenience constructor for ROS messages
   *
   * Implicitly normalizes the point's orientation quaternion.
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
  Eigen::Vector3d rot() const
  {
    Eigen::AngleAxisd a(q);
    return a.axis() * a.angle();
  };

  /**
   * @brief Stream operator for testing and debugging
   *
   * @param os The output stream
   * @param state The CartesianState
   *
   * @return Reference to the stream for chaining
   */
  friend std::ostream& operator<<(std::ostream& os, const CartesianState& state);

  // Pose
  Eigen::Vector3d p;     ///< position
  Eigen::Quaterniond q;  ///< rotation

  // Spatial velocity in body (waypoint) coordinates
  Eigen::Vector3d v;  ///< linear velocity, \f$ v \f$
  Eigen::Vector3d w;  ///< angular velocity, \f$ \omega \f$

  // Spatial acceleration in body (waypoint) coordinates
  Eigen::Vector3d v_dot;  ///< linear acceleration, \f$ \dot{v} \f$
  Eigen::Vector3d w_dot;  ///< angular acceleration, \f$ \dot{\omega} \f$
};

}  // namespace ros_controllers_cartesian
