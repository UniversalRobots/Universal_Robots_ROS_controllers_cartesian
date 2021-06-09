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
/*!\file    cartesian_trajectory_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2021/01/24
 *
 */
//-----------------------------------------------------------------------------

#include <pluginlib/class_list_macros.h>
#include <cartesian_trajectory_controller/cartesian_trajectory_controller.h>
#include <cartesian_interface/cartesian_command_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include "hardware_interface/joint_state_interface.h"

namespace pose_controllers
{
using CartesianTrajectoryController =
    cartesian_trajectory_controller::CartesianTrajectoryController<ros_controllers_cartesian::PoseCommandInterface>;
}

namespace twist_controllers
{
using CartesianTrajectoryController =
    cartesian_trajectory_controller::CartesianTrajectoryController<ros_controllers_cartesian::TwistCommandInterface>;
}

namespace position_controllers
{
using CartesianTrajectoryController =
    cartesian_trajectory_controller::CartesianTrajectoryController<hardware_interface::PositionJointInterface>;
}

namespace velocity_controllers
{
using CartesianTrajectoryController =
    cartesian_trajectory_controller::CartesianTrajectoryController<hardware_interface::VelocityJointInterface>;
}

namespace cartesian_trajectory_publisher
{
using CartesianTrajectoryPublisher =
    cartesian_trajectory_controller::CartesianTrajectoryController<hardware_interface::JointStateInterface>;
}

PLUGINLIB_EXPORT_CLASS(pose_controllers::CartesianTrajectoryController, controller_interface::ControllerBase)

/* Not yet implemented.
PLUGINLIB_EXPORT_CLASS(twist_controllers::CartesianTrajectoryController,
                       controller_interface::ControllerBase)
*/

PLUGINLIB_EXPORT_CLASS(position_controllers::CartesianTrajectoryController, controller_interface::ControllerBase)
/* Not yet implemented
PLUGINLIB_EXPORT_CLASS(velocity_controllers::CartesianTrajectoryController,
                       controller_interface::ControllerBase)
*/

PLUGINLIB_EXPORT_CLASS(cartesian_trajectory_publisher::CartesianTrajectoryPublisher,
                       controller_interface::ControllerBase)
