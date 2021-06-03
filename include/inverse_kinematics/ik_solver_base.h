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

namespace cartesian_ros_control
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

}
