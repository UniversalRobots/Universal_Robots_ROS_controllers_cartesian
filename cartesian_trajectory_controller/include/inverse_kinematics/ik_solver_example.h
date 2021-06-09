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
/*!\file    ik_solver_example.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2021/05/12
 *
 */
//-----------------------------------------------------------------------------

#pragma once

#include <inverse_kinematics/ik_solver_base.h>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chain.hpp>
#include <memory>

namespace ros_controllers_cartesian
{
/**
 * @brief A wrapper around KDL's Levenberg Marquardt solver
 *
 * This is the default Inverse Kinematics (IK) solver for the
 * cartesian_trajectory_controller.
 */
class ExampleIKSolver : public IKSolver
{
public:
  ExampleIKSolver(){};
  ~ExampleIKSolver(){};

  /**
   * @brief Initialize the solver
   *
   * Only the kinematics chain is used.
   *
   */
  bool init(const KDL::Chain& robot_chain, ros::NodeHandle&, ros::NodeHandle&) override
  {
    robot_chain_ = robot_chain;
    lma_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(robot_chain_);
    return true;
  };

  /**
   * @brief Compute Inverse Kinematics with KDL's Levenberg Marquardt solver.
   *
   */
  virtual int cartToJnt(const KDL::JntArray& q_init, const KDL::Frame& goal, KDL::JntArray& q_out) override
  {
    return lma_solver_->CartToJnt(q_init, goal, q_out);
  };

private:
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> lma_solver_;
  KDL::Chain robot_chain_;
};
}  // namespace ros_controllers_cartesian
