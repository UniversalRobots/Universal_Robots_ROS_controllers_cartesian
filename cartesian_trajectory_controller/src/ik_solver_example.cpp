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
/*!\file    ik_solver_example.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2021/05/12
 *
 */
//-----------------------------------------------------------------------------

#include <inverse_kinematics/ik_solver_example.h>

// Pluginlib
#include <pluginlib/class_list_macros.h>

/**
 * \class ros_controllers_cartesian::ExampleIKSolver
 *
 * You may explicitly specify this solver with \a "example_solver" as \a
 * ik_solver in the controllers.yaml configuration file:
 *
 * \code{.yaml}
 * <name_of_your_controller>:
 *     type: "position_controllers/CartesianTrajectoryController"
 *     ik_solver: "example_solver"
 *     ...
 * \endcode
 *
 */
PLUGINLIB_EXPORT_CLASS(ros_controllers_cartesian::ExampleIKSolver, ros_controllers_cartesian::IKSolver)
