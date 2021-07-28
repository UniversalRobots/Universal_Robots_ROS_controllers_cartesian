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
/*!\file    cartesian_trajectory_controller.hpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2021/01/24
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_trajectory_controller/cartesian_trajectory_controller.h>
#include "hardware_interface/robot_hw.h"

namespace cartesian_trajectory_controller
{
template <class HWInterface>
bool CartesianTrajectoryController<HWInterface>::init(hardware_interface::RobotHW* hw, ros::NodeHandle& nh,
                                                      ros::NodeHandle& controller_nh)
{
  if (!ControlPolicy::init(hw, nh, controller_nh))
  {
    return false;
  }

  // Use speed scaling interface if available
  auto speed_scaling_interface = hw->get<scaled_controllers::SpeedScalingInterface>();
  if (!speed_scaling_interface)
  {
    ROS_INFO_STREAM(controller_nh.getNamespace() << ": Your RobotHW seems not to provide speed scaling. Starting "
                                                    "without this feature.");
    speed_scaling_ = nullptr;
  }
  else
  {
    speed_scaling_ = std::make_unique<scaled_controllers::SpeedScalingHandle>(speed_scaling_interface->getHandle("speed"
                                                                                                                 "_scal"
                                                                                                                 "ing_"
                                                                                                                 "facto"
                                                                                                                 "r"));
  }

  // Action server
  action_server_.reset(new actionlib::SimpleActionServer<cartesian_control_msgs::FollowCartesianTrajectoryAction>(
      controller_nh, "follow_cartesian_trajectory",
      std::bind(&CartesianTrajectoryController::executeCB, this, std::placeholders::_1), false));

  action_server_->registerPreemptCallback(std::bind(&CartesianTrajectoryController::preemptCB, this));

  action_server_->start();

  return true;
}

template <class HWInterface>
void CartesianTrajectoryController<HWInterface>::starting(const ros::Time& time)
{
  // Start where we are
  ControlPolicy::updateCommand(ControlPolicy::getState());
}

template <class HWInterface>
void CartesianTrajectoryController<HWInterface>::stopping(const ros::Time& time)
{
  if (action_server_->isActive())
  {
    // Set canceled flag in the action result
    action_server_->setPreempted();
  }
}

template <class HWInterface>
void CartesianTrajectoryController<HWInterface>::update(const ros::Time& time, const ros::Duration& period)
{
  if (action_server_->isActive() && !done_.load())
  {
    // Apply speed scaling if available.
    const double factor = (speed_scaling_) ? *speed_scaling_->getScalingFactor() : 1.0;
    trajectory_duration_.now += period * factor;

    // Sample the Cartesian trajectory's target state and command that to
    // the control policy.
    if (trajectory_duration_.now < trajectory_duration_.end)
    {
      std::lock_guard<std::mutex> lock_trajectory(lock_);

      ros_controllers_cartesian::CartesianState desired;
      trajectory_.sample(trajectory_duration_.now.toSec(), desired);

      ControlPolicy::updateCommand(desired);

      // Give feedback
      auto actual = ControlPolicy::getState();
      auto error = desired - actual;

      cartesian_control_msgs::FollowCartesianTrajectoryFeedback f;
      auto now = trajectory_duration_.now.toSec();
      f.desired = desired.toMsg(now);
      f.actual = actual.toMsg(now);
      f.error = error.toMsg(now);

      action_server_->publishFeedback(f);

      // Check tolerances and set terminal conditions for the
      // action server if special criteria are met.
      monitorExecution(error);
    }
    else  // Time is up. Check goal tolerances and set terminal state.
    {
      timesUp();
    }
  }
}

template <class HWInterface>
void CartesianTrajectoryController<HWInterface>::executeCB(
    const cartesian_control_msgs::FollowCartesianTrajectoryGoalConstPtr& goal)
{
  // Upon entering this callback, the simple action server has already
  // preempted the previously active goal (if any) and has accepted the new goal.

  if (!this->isRunning())
  {
    ROS_ERROR("Can't accept new action goals. Controller is not running.");
    cartesian_control_msgs::FollowCartesianTrajectoryResult result;
    result.error_code = cartesian_control_msgs::FollowCartesianTrajectoryResult::INVALID_GOAL;
    action_server_->setAborted(result);
    return;
  }

  path_tolerances_ = goal->path_tolerance;
  goal_tolerances_ = goal->goal_tolerance;

  // Start where we are by adding the current state as first trajectory
  // waypoint.
  auto state = ControlPolicy::getState();
  {
    std::lock_guard<std::mutex> lock_trajectory(lock_);

    cartesian_control_msgs::CartesianTrajectory traj = goal->trajectory;
    traj.points.insert(traj.points.begin(), state.toMsg(0));  // start time zero

    if (!trajectory_.init(traj))
    {
      ROS_ERROR("Action goal has invalid trajectory.");
      cartesian_control_msgs::FollowCartesianTrajectoryResult result;
      result.error_code = cartesian_control_msgs::FollowCartesianTrajectoryResult::INVALID_GOAL;
      action_server_->setAborted(result);
      return;
    }
  }

  // Time keeping
  trajectory_duration_.now = ros::Duration(0.0);
  trajectory_duration_.end = goal->trajectory.points.back().time_from_start + goal->goal_time_tolerance;

  done_ = false;

  while (!done_.load())
  {
    ros::Duration(0.01).sleep();
  }
}

template <class HWInterface>
void CartesianTrajectoryController<HWInterface>::preemptCB()
{
  cartesian_control_msgs::FollowCartesianTrajectoryResult result;
  result.error_string = "preempted";
  action_server_->setPreempted(result);

  done_ = true;
}

template <class HWInterface>
void CartesianTrajectoryController<HWInterface>::timesUp()
{
  using Result = cartesian_control_msgs::FollowCartesianTrajectoryResult;
  Result result;

  // When time is over, sampling gives us the last waypoint.
  ros_controllers_cartesian::CartesianState goal;
  {
    std::lock_guard<std::mutex> lock_trajectory(lock_);
    trajectory_.sample(trajectory_duration_.now.toSec(), goal);
  }

  // TODO: What should happen when speed scaling was active?
  //       Only check position and orientation in that case?
  // Address this once we know more edge cases during beta testing.

  // Check if goal was reached.
  // Abort if any of the dimensions exceeds its goal tolerance
  auto error = goal - ControlPolicy::getState();

  if (!withinTolerances(error, goal_tolerances_))
  {
    result.error_code = Result::GOAL_TOLERANCE_VIOLATED;
    action_server_->setAborted(result);
  }
  else  // Succeed
  {
    result.error_code = Result::SUCCESSFUL;
    action_server_->setSucceeded(result);
  }

  done_ = true;
}

template <class HWInterface>
void CartesianTrajectoryController<HWInterface>::monitorExecution(
    const ros_controllers_cartesian::CartesianState& error)
{
  if (!withinTolerances(error, path_tolerances_))
  {
    using Result = cartesian_control_msgs::FollowCartesianTrajectoryResult;
    Result result;
    result.error_code = Result::PATH_TOLERANCE_VIOLATED;
    action_server_->setAborted(result);
    done_ = true;
  }
}

template <class HWInterface>
bool CartesianTrajectoryController<HWInterface>::withinTolerances(
    const ros_controllers_cartesian::CartesianState& error, const cartesian_control_msgs::CartesianTolerance& tolerance)
{
  // Uninitialized tolerances do not need checking
  cartesian_control_msgs::CartesianTolerance uninitialized;
  std::stringstream str_1;
  std::stringstream str_2;
  str_1 << tolerance;
  str_2 << uninitialized;

  if (str_1.str() == str_2.str())
  {
    return true;
  }

  auto not_within_limits = [](const auto& a, const auto& b) { return a.x() > b.x || a.y() > b.y || a.z() > b.z; };

  // Check each individual dimension separately.
  if (not_within_limits(error.p, tolerance.position_error) ||
      not_within_limits(error.rot(), tolerance.orientation_error) ||
      not_within_limits(error.v, tolerance.twist_error.linear) ||
      not_within_limits(error.w, tolerance.twist_error.angular) ||
      not_within_limits(error.v_dot, tolerance.acceleration_error.linear) ||
      not_within_limits(error.w_dot, tolerance.acceleration_error.angular))
  {
    return false;
  }

  return true;
}

}  // namespace cartesian_trajectory_controller
