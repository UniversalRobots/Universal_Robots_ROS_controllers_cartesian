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
/*!\file    cartesian_trajectory.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2021/01/14
 *
 */
//-----------------------------------------------------------------------------

#pragma once

#include <cartesian_trajectory_interpolation/cartesian_trajectory_segment.h>
#include <cartesian_trajectory_interpolation/cartesian_state.h>
#include <cartesian_control_msgs/CartesianTrajectory.h>
#include <vector>

namespace ros_controllers_cartesian
{
/**
 * @brief A class for Cartesian trajectory representation and interpolation
 *
 * It's meant to be used inside ROS controllers to wrap the complexity of
 * trajectory interpolation.  Initialize instances of this helper with
 * Cartesian ROS trajectories and sample \a CartesianState at specific time
 * steps for robot control.
 *
 */
class CartesianTrajectory
{
public:
  CartesianTrajectory() = default;

  /**
   * @brief Construct from ROS messages
   *
   * Calls init() and throws std::invalid_argument if that fails.
   *
   * @param ros_trajectory The Cartesian trajectory composed with ROS message types
   */
  CartesianTrajectory(const cartesian_control_msgs::CartesianTrajectory& ros_trajectory);

  virtual ~CartesianTrajectory(){};

  /**
   * @brief Initialize from ROS message
   *
   * \note The first waypoint is expected to be the current state with time_from_start == 0.
   *
   * @param ros_trajectory The Cartesian trajectory composed with ROS message types
   *
   * @return True if \b ros_trajectory has increasing waypoints in time, else false.
   */
  bool init(const cartesian_control_msgs::CartesianTrajectory& ros_trajectory);

  /**
   * @brief Sample a trajectory at a specified time
   *
   * The trajectory's waypoints are interpolated with splines (two-point
   * polynomials) that are uniquely defined between each two waypoints and
   * that scale with the fields given:
   *
   * - Only pose = linear interpolation
   * - Pose and velocity = cubic interpolation
   * - Pose, velocity and acceleration = quintic interpolation
   *
   * If this trajectory's waypoints have velocity and acceleration
   * setpoints, \b state also contains the current velocity and
   * acceleration, respectively.  A typical application is using the
   * sampled state as reference for robot control.
   *
   * @param time Time at which to sample the trajectory
   * @param state Cartesian state at \b time
   */
  void sample(const CartesianTrajectorySegment::Time& time, CartesianState& state);

private:
  std::vector<CartesianTrajectorySegment> trajectory_data_;
};

}  // namespace ros_controllers_cartesian
