////////////////////////////////////////////////////////////////////////////////
// Copyright 2020 FZI Research Center for Information Technology
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

// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

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
    bool CartesianTrajectoryController<HWInterface>::
    init(hardware_interface::RobotHW* hw, ros::NodeHandle& nh, ros::NodeHandle& controller_nh)
    {

      // Action server
      action_server_.reset(new actionlib::SimpleActionServer<cartesian_control_msgs::FollowCartesianTrajectoryAction>(
            controller_nh,
            "follow_cartesian_trajectory",
            std::bind(&CartesianTrajectoryController::executeCB, this, std::placeholders::_1),
            false));

      action_server_->registerPreemptCallback(
          std::bind(&CartesianTrajectoryController::preemptCB, this));

      action_server_->start();

      return true;
    }

  template <class HWInterface>
    void CartesianTrajectoryController<HWInterface>::starting(const ros::Time& time)
    {
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
    void CartesianTrajectoryController<HWInterface>::update(const ros::Time& time,
        const ros::Duration& period)
    {
      if (action_server_->isActive() && !done_.load())
      {
        trajectory_duration_.now += period;


        // Sample the Cartesian trajectory's target state and command that to
        // the control policy.
        if (trajectory_duration_.now < trajectory_duration_.end)
        {
          std::lock_guard<std::mutex> lock_trajectory(lock_);

          cartesian_ros_control::CartesianState target;
          trajectory_.sample(trajectory_duration_.now.toSec(), target);

          ControlPolicy::updateCommand(target);
        }
        else // Time is up. Check goal tolerances and set terminal state.
        {
          // TODO
          done_ = true;
        }

        // TODO: Implement feedback
        // cartesian_control_msgs::FollowCartesianTrajectoryFeedback f;
        // action_server_->publishFeedback(f);
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

      // TODO: Check if trajectory is valid

      // Start where we are by adding the current state as first trajectory
      // waypoint.
      auto state = ControlPolicy::getState();
      {
        std::lock_guard<std::mutex> lock_trajectory(lock_);

        cartesian_control_msgs::CartesianTrajectory traj = goal->trajectory;
        traj.points.insert(traj.points.begin(), state.toMsg(0));  // start time zero

        trajectory_.init(traj);
      }

      // Time keeping
      trajectory_duration_.now = ros::Duration(0.0);
      trajectory_duration_.end =
        goal->trajectory.points.back().time_from_start + goal->goal_time_tolerance;

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


}
