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

namespace cartesian_ros_control
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
}


