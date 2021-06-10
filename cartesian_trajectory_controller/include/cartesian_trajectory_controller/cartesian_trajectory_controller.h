// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2020 FZI Forschungszentrum Informatik
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
/*!\file    cartesian_trajectory_controller.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2021/01/24
 *
 */
//-----------------------------------------------------------------------------

#pragma once

#include <cartesian_trajectory_controller/control_policies.h>
#include <cartesian_trajectory_interpolation/cartesian_trajectory.h>
#include <cartesian_trajectory_interpolation/cartesian_state.h>
#include <cartesian_control_msgs/FollowCartesianTrajectoryAction.h>
#include <cartesian_control_msgs/CartesianTolerance.h>
#include <atomic>
#include <mutex>
#include <actionlib/server/simple_action_server.h>
#include <speed_scaling_interface/speed_scaling_interface.h>

namespace cartesian_trajectory_controller
{
template <class HWInterface>
class CartesianTrajectoryController : public ros_controllers_cartesian::ControlPolicy<HWInterface>
{
public:
  CartesianTrajectoryController() : ros_controllers_cartesian::ControlPolicy<HWInterface>(){};

  virtual ~CartesianTrajectoryController(){};

  bool init(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  void starting(const ros::Time& time) override;

  void stopping(const ros::Time& time) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

  void executeCB(const cartesian_control_msgs::FollowCartesianTrajectoryGoalConstPtr& goal);

  void preemptCB();

protected:
  using ControlPolicy = ros_controllers_cartesian::ControlPolicy<HWInterface>;

  struct TrajectoryDuration
  {
    TrajectoryDuration() : now(0.0), end(0.0)
    {
    }

    ros::Duration end;  ///< Planned target duration of the current action.
    ros::Duration now;  ///< Current duration of the current action.
  };

  void timesUp();

  void monitorExecution(const ros_controllers_cartesian::CartesianState& error);

  bool withinTolerances(const ros_controllers_cartesian::CartesianState& error,
                        const cartesian_control_msgs::CartesianTolerance& tolerance);

private:
  std::unique_ptr<scaled_controllers::SpeedScalingHandle> speed_scaling_;
  std::unique_ptr<actionlib::SimpleActionServer<cartesian_control_msgs::FollowCartesianTrajectoryAction> >
      action_server_;
  std::atomic<bool> done_;
  std::mutex lock_;
  ros_controllers_cartesian::CartesianTrajectory trajectory_;
  TrajectoryDuration trajectory_duration_;
  cartesian_control_msgs::CartesianTolerance path_tolerances_;
  cartesian_control_msgs::CartesianTolerance goal_tolerances_;
};

}  // namespace cartesian_trajectory_controller

#include <cartesian_trajectory_controller/cartesian_trajectory_controller.hpp>
