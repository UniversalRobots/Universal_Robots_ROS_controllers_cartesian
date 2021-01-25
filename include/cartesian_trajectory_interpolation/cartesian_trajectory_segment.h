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
/*!\file    cartesian_trajectory_segment.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2021/01/14
 *
 */
//-----------------------------------------------------------------------------

#pragma once

#include <trajectory_interface/quintic_spline_segment.h>
#include <trajectory_interface/pos_vel_acc_state.h>
#include <cartesian_trajectory_interpolation/cartesian_state.h>

namespace cartesian_ros_control
{

  /**
   * @brief 7-dimensional quintic spline segment
   *
   * This two-point segment is uniquely defined by its start and end
   * cartesian_ros_control::SplineState.
   */
  using QuinticSplineSegment = trajectory_interface::QuinticSplineSegment<double>;


  /**
   * @brief Cartesian segment between two trajectory waypoints.
   *
   * Various of these segments form a Cartesian trajectory.
   *
   * They use the \ref QuinticSplineSegment implementation from the
   * <a href="https://github.com/ros-controls/ros_controllers/blob/noetic-devel/joint_trajectory_controller/include/trajectory_interface/quintic_spline_segment.h">joint_trajectory_controller</a>
   * package.  For each of the seven Cartesian
   * dimensions, they implement linear, cubic or quintic spline interpolation,
   * depending on the waypoints specification by parameterizing unique
   * polynomials between the two waypoints.
   *
   * This wrapper takes care of converting Cartesian pose, velocity and
   * acceleration setpoints into quaternion quantities for specifying spline
   * boundary conditions (during initialization), and re-converting the
   * interpolated values back into Cartesian quantities that can be commanded
   * to robot control (upon sampling).
   *
   */
  class CartesianTrajectorySegment : public QuinticSplineSegment
  {

    public:
      /**
       * @brief Relative time in seconds
       */
      using Time = QuinticSplineSegment::Time;

      using State = CartesianState;

      /**
       * @brief State of a 7-dimensional Spline
       *
       * The first three dimensions represent Cartesian translation and the last
       * four represent the rotation quaternion.  Each dimension has its own
       * position, velocity and acceleration value.
       *
       */
      using SplineState = QuinticSplineSegment::State;


      CartesianTrajectorySegment() = delete;

      virtual ~CartesianTrajectorySegment(){};

      /**
       * @brief Construct a Cartesian trajectory segment from stat and end state.
       *
       * @param start_time Time in seconds when this segment starts
       * @param start_state CartesianState at start time
       * @param end_time Time in seconds when this segment ends
       * @param end_state CartesianState at end time
       */
      CartesianTrajectorySegment(const Time&  start_time,
          const CartesianState& start_state,
          const Time&  end_time,
          const CartesianState& end_state);

      /**
       * @brief Sample the CartesianState at the given time
       *
       * \note Will use \ref QuinticSplineSegment::sample internally with the following characteristics: \n
       * Within the <tt>[start_time, end_time]</tt> interval, spline
       * interpolation takes place, outside it this method will output the
       * start/end states with zero velocity and acceleration.
       *
       *
       * @param time The time at which to sample
       * @param state The CartesianState at the requested \b time
       */
      void sample(const Time& time, CartesianState& state) const;

      /**
       * @brief Convert a CartesianState into a cartesian_ros_control::SplineState
       *
       * The computation of quaternion velocities and accelerations from
       * Cartesian angular velocities and accelerations is based on
       * <a href="https://math.stackexchange.com/questions/1792826/estimate-angular-velocity-and-acceleration-from-a-sequence-of-rotations">this blog post</a>.
       * \note SplineState has the velocities and accelerations
       * given in the body-local reference frame that is implicitly defined
       * by \b state's pose. 
       *
       * @param state The CartesianState to convert
       *
       * @return The content of \b state in cartesian_ros_control::SplineState notation
       */
      SplineState convert(const CartesianState& state) const;

    private:
  };

}
