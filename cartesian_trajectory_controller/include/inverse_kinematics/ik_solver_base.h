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
/*!\file    ik_solver_base.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2021/05/12
 *
 */
//-----------------------------------------------------------------------------

#pragma once

#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <ros/ros.h>

namespace ros_controllers_cartesian
{
/**
 * @brief Base class for Inverse Kinematics (IK) solvers
 *
 * This base class is meant to provide an interface for custom IK implementations.
 * The joint-based control policies in the Cartesian trajectory
 * controller will need some form of IK solver. This allows you to implement
 * your own (possibly more advanced) algorithm. For instance, you may wish
 * to consider collision checking by reacting ad-hoc to objects
 * that additional sensors perceive in your environment.
 */
class IKSolver
{
public:
  IKSolver(){};
  virtual ~IKSolver(){};

  /**
   * @brief Initialize the solver
   *
   * @param robot_chain Representation of the robot kinematics
   *
   * @param root_nh A NodeHandle in the root of the controller manager namespace.
   *
   * @param controller_nh A NodeHandle in the namespace of the controller.
   * This is where the Cartesian trajectory controller-specific configuration resides.
   *
   * @return True if initialization was successful.
   */
  virtual bool init(const KDL::Chain& robot_chain, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) = 0;

  /**
   * @brief Compute Inverse Kinematics
   *
   * @param q_init Vector of initial joint positions
   * @param goal Goal pose with respect to the robot base
   * @param q_out Vector of suitable joint positions
   *
   * @return 0 if successful. Derived classes implement specializations.
   */
  virtual int cartToJnt(const KDL::JntArray& q_init, const KDL::Frame& goal, KDL::JntArray& q_out) = 0;
};

}  // namespace ros_controllers_cartesian
