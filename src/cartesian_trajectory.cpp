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
/*!\file    cartesian_trajectory.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2021/01/21
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_trajectory_interpolation/cartesian_trajectory.h>
#include <trajectory_interface/trajectory_interface.h>

namespace cartesian_ros_control
{

  void CartesianTrajectory::sample(const CartesianTrajectorySegment::Time& time, CartesianState& state)
  {
    trajectory_interface::sample(trajectory_data_, time, state);
  }

  CartesianTrajectory::CartesianTrajectory(const cartesian_control_msgs::CartesianTrajectory& ros_trajectory)
  {
    if (!init(ros_trajectory))
    {
      throw std::invalid_argument("Trajectory not valid");
    };
  }

  bool CartesianTrajectory::init(const cartesian_control_msgs::CartesianTrajectory& ros_trajectory)
  {
    trajectory_data_.clear();

    // Loop through the waypoints and build trajectory segments from each two
    // neighboring pairs.
    for (auto i = ros_trajectory.points.begin(); std::next(i) < ros_trajectory.points.end(); ++i)
    {
      CartesianTrajectorySegment s(
          i->time_from_start.toSec(), CartesianState(*i),
          std::next(i)->time_from_start.toSec(), CartesianState(*std::next(i))
          );
      trajectory_data_.push_back(s);
    }
    return true;
  }
}
