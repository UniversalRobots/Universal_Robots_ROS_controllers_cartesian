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
/*!\file    control_policies.hpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2021/01/24
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_trajectory_controller/cartesian_trajectory_controller.h>

#include <memory>

// KDL
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include "kdl/frames.hpp"
#include "kdl/framevel.hpp"
#include "kdl/jntarrayvel.hpp"

// URDF
#include <urdf/model.h>

namespace cartesian_ros_control
{

  bool ControlPolicy<hardware_interface::PositionJointInterface>::init(hardware_interface::RobotHW* hw,
      ros::NodeHandle& root_nh,
      ros::NodeHandle& controller_nh)
  {
    std::string robot_description;
    std::vector<std::string> joint_names;
    std::string tip;
    std::string base;
    urdf::Model robot_model;
    KDL::Tree   robot_tree;

    const std::string ns = controller_nh.getNamespace();

    // Preconditions
    auto* pos_hw = hw->get<hardware_interface::PositionJointInterface>();
    if (!pos_hw)
    {
      ROS_ERROR_STREAM(ns << ": No PositionJointInterface found.");
      return false;
    }

    // Joint control interfaces
    if (!controller_nh.getParam("joints", joint_names))
    {
      ROS_ERROR_STREAM(ns << ": Failed to load joints from parameter server");
      return false;
    }
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
      joint_pos_handles_.push_back(pos_hw->getHandle(joint_names[i]));
    }

    // Manipulator specific configuration
    if (!root_nh.getParam("/robot_description", robot_description))
    {
      ROS_ERROR_STREAM(ns << ": Failed to load /robot_description from parameter server");
      return false;
    }
    if (!controller_nh.getParam("base", base))
    {
      ROS_ERROR_STREAM(ns << ": Failed to load base from parameter server");
      return false;
    }
    if (!controller_nh.getParam("tip", tip))
    {
      ROS_ERROR_STREAM(ns << ": Failed to load tip from parameter server");
      return false;
    }

    // Construction of kinematics
    if (!robot_model.initString(robot_description))
    {
      ROS_ERROR_STREAM(ns << ": Failed to parse urdf model from robot_description");
      return false;
    }
    if (!kdl_parser::treeFromUrdfModel(robot_model, robot_tree))
    {
      ROS_ERROR_STREAM(ns << ": Failed to parse KDL tree from urdf model");
      return false;
    }
    if (!robot_tree.getChain(base, tip, robot_chain_))
    {
      ROS_ERROR_STREAM(ns << ": Failed to parse robot chain from urdf model.");
      return false;
    }

    fk_solver_ = std::make_unique<KDL::ChainFkSolverVel_recursive>(robot_chain_);
    ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(robot_chain_);

    return true;
  }

  CartesianState ControlPolicy<hardware_interface::PositionJointInterface>::getState() const
  {
    const size_t size = joint_pos_handles_.size();

    KDL::JntArrayVel current(size);
    KDL::FrameVel pose;

    // Compute forward kinematics
    for (size_t i = 0; i < size; ++i)
    {
      current.q(i) = joint_pos_handles_[i].getPosition();
      current.qdot(i) = joint_pos_handles_[i].getVelocity();
    }
    fk_solver_->JntToCart(current, pose);

    // Pose
    CartesianState state;
    state.p.x() = pose.value().p.x();
    state.p.y() = pose.value().p.y();
    state.p.z() = pose.value().p.z();
    pose.value().M.GetQuaternion(state.q.x(), state.q.y(), state.q.z(), state.q.w());

    // Velocity
    state.v.x() = pose.deriv().vel.x();;
    state.v.y() = pose.deriv().vel.y();;
    state.v.z() = pose.deriv().vel.z();;
    state.w.x() = pose.deriv().rot.x();;
    state.w.y() = pose.deriv().rot.y();;
    state.w.z() = pose.deriv().rot.z();;

    return state;
  };

  void ControlPolicy<hardware_interface::PositionJointInterface>::updateCommand(const CartesianState& cmd)
  {
    const size_t size = joint_pos_handles_.size();

    KDL::JntArray current(size);
    KDL::JntArray target(size);
    KDL::Frame goal;

    goal.p[0] = cmd.p.x();
    goal.p[1] = cmd.p.y();
    goal.p[2] = cmd.p.z();
    goal.M = KDL::Rotation::Quaternion(cmd.q.x(), cmd.q.y(), cmd.q.z(), cmd.q.w());

    // Start where we are
    for (size_t i = 0; i < size; ++i)
    {
      current(i) = joint_pos_handles_[i].getPosition();
    }

    // Compute inverse kinematics
    ik_solver_->CartToJnt(current, goal, target);

    // Command each joint
    for (size_t i = 0; i < size; ++i)
    {
      joint_pos_handles_[i].setCommand(target(i));
    }
  }
}
