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
#include <cartesian_interface/speed_scaling_interface.h>

namespace cartesian_trajectory_controller
{

  template <class HWInterface>
    class CartesianTrajectoryController : public cartesian_ros_control::ControlPolicy<HWInterface>
  {

    public:
      CartesianTrajectoryController()
        : cartesian_ros_control::ControlPolicy<HWInterface>()
      {};

      virtual ~CartesianTrajectoryController(){};

      bool init(hardware_interface::RobotHW* hw,
          ros::NodeHandle& root_nh,
          ros::NodeHandle& controller_nh) override;

      void starting(const ros::Time& time) override;

      void stopping(const ros::Time& time) override;

      void update(const ros::Time& time, const ros::Duration& period) override;

      void executeCB(const cartesian_control_msgs::FollowCartesianTrajectoryGoalConstPtr& goal);

      void preemptCB();

    protected:
      using ControlPolicy = cartesian_ros_control::ControlPolicy<HWInterface>;

      struct TrajectoryDuration
      {
        TrajectoryDuration() : now(0.0), end(0.0) {}

        ros::Duration end; ///< Planned target duration of the current action.
        ros::Duration now; ///< Current duration of the current action.
      };

      void timesUp();

      void monitorExecution(const cartesian_ros_control::CartesianState& error);

      bool withinTolerances(const cartesian_ros_control::CartesianState& error,
                            const cartesian_control_msgs::CartesianTolerance& tolerance);

    private:
      std::unique_ptr<hardware_interface::SpeedScalingHandle> speed_scaling_;
      std::unique_ptr<actionlib::SimpleActionServer<cartesian_control_msgs::FollowCartesianTrajectoryAction> >
        action_server_;
      std::atomic<bool> done_;
      std::mutex lock_;
      cartesian_ros_control::CartesianTrajectory trajectory_;
      TrajectoryDuration trajectory_duration_;
      cartesian_control_msgs::CartesianTolerance path_tolerances_;
      cartesian_control_msgs::CartesianTolerance goal_tolerances_;
  };

}

#include <cartesian_trajectory_controller/cartesian_trajectory_controller.hpp>
