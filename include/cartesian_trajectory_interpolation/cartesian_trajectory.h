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


namespace cartesian_ros_control
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
       * @return True if \b ros_trajectory is valid, else false.
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

}
