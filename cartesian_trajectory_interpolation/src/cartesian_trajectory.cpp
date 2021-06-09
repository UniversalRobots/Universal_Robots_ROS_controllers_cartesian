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
/*!\file    cartesian_trajectory.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2021/01/21
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_trajectory_interpolation/cartesian_trajectory.h>
#include <trajectory_interface/trajectory_interface.h>

namespace ros_controllers_cartesian
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
    // Waypoints' time from start must strictly increase
    if (i->time_from_start.toSec() >= std::next(i)->time_from_start.toSec())
    {
      return false;
    }
    CartesianTrajectorySegment s(i->time_from_start.toSec(), CartesianState(*i), std::next(i)->time_from_start.toSec(),
                                 CartesianState(*std::next(i)));
    trajectory_data_.push_back(s);
  }
  return true;
}
}  // namespace ros_controllers_cartesian
