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
/*!\file    control_policies.hpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2021/01/24
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_trajectory_controller/cartesian_trajectory_controller.h>

#include <memory>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

// KDL
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include "cartesian_interface/cartesian_command_interface.h"
#include "cartesian_interface/cartesian_state_handle.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "kdl/chainiksolvervel_wdls.hpp"
#include "kdl/frames.hpp"
#include "kdl/framevel.hpp"
#include "kdl/jntarrayvel.hpp"

// URDF
#include <urdf/model.h>

namespace ros_controllers_cartesian
{
template <class HWInterface, class HandleType>
bool JointBasedController<HWInterface, HandleType>::init(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh,
                                                         ros::NodeHandle& controller_nh)
{
  std::string robot_description;
  std::vector<std::string> joint_names;
  urdf::Model robot_model;
  KDL::Tree robot_tree;

  const std::string ns = controller_nh.getNamespace();

  // Preconditions
  auto* hw_interface = hw->get<HWInterface>();
  if (!hw_interface)
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
    joint_handles_.push_back(hw_interface->getHandle(joint_names[i]));
  }

  // Manipulator specific configuration
  if (!root_nh.getParam("robot_description", robot_description))
  {
    ROS_ERROR_STREAM(ns << ": Failed to load robot_description from parameter server");
    return false;
  }
  if (!controller_nh.getParam("base", robot_base_))
  {
    ROS_ERROR_STREAM(ns << ": Failed to load base from parameter server");
    return false;
  }
  if (!controller_nh.getParam("tip", robot_tip_))
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
  if (!robot_tree.getChain(robot_base_, robot_tip_, robot_chain_))
  {
    ROS_ERROR_STREAM(ns << ": Failed to parse robot chain from urdf model.");
    return false;
  }

  fk_solver_ = std::make_unique<KDL::ChainFkSolverVel_recursive>(robot_chain_);

  return true;
}

template <class HWInterface, class HandleType>
CartesianState JointBasedController<HWInterface, HandleType>::getState() const
{
  const size_t size = joint_handles_.size();

  KDL::JntArrayVel current(size);
  KDL::FrameVel pose;

  // Compute forward kinematics
  for (size_t i = 0; i < size; ++i)
  {
    current.q(i) = joint_handles_[i].getPosition();
    current.qdot(i) = joint_handles_[i].getVelocity();
  }
  fk_solver_->JntToCart(current, pose);

  // Pose
  CartesianState state;
  state.p.x() = pose.value().p.x();
  state.p.y() = pose.value().p.y();
  state.p.z() = pose.value().p.z();
  pose.value().M.GetQuaternion(state.q.x(), state.q.y(), state.q.z(), state.q.w());

  // Velocity
  state.v.x() = pose.deriv().vel.x();
  ;
  state.v.y() = pose.deriv().vel.y();
  ;
  state.v.z() = pose.deriv().vel.z();
  ;
  state.w.x() = pose.deriv().rot.x();
  ;
  state.w.y() = pose.deriv().rot.y();
  ;
  state.w.z() = pose.deriv().rot.z();
  ;

  return state;
};

//--------------------------------------------------------------------------------
// Cartesian pose
//--------------------------------------------------------------------------------

bool ControlPolicy<ros_controllers_cartesian::PoseCommandInterface>::init(hardware_interface::RobotHW* hw,
                                                                          ros::NodeHandle& root_nh,
                                                                          ros::NodeHandle& controller_nh)
{
  const std::string ns = controller_nh.getNamespace();

  // Preconditions
  auto* cmd_interface = hw->get<ros_controllers_cartesian::PoseCommandInterface>();
  if (!cmd_interface)
  {
    ROS_ERROR_STREAM(ns << ": No PoseCommandInterface found.");
    return false;
  }
  if (!controller_nh.getParam("base", base_))
  {
    ROS_ERROR_STREAM(ns << ": Failed to load base from parameter server");
    return false;
  }
  if (!controller_nh.getParam("tip", tip_))
  {
    ROS_ERROR_STREAM(ns << ": Failed to load tip from parameter server");
    return false;
  }

  handle_ = cmd_interface->getHandle(tip_);

  return true;
}

void ControlPolicy<ros_controllers_cartesian::PoseCommandInterface>::updateCommand(const CartesianState& cmd)
{
  geometry_msgs::Pose target;

  target.position.x = cmd.p.x();
  target.position.y = cmd.p.y();
  target.position.z = cmd.p.z();
  target.orientation.x = cmd.q.x();
  target.orientation.y = cmd.q.y();
  target.orientation.z = cmd.q.z();
  target.orientation.w = cmd.q.w();

  handle_.setCommand(target);
}

CartesianState ControlPolicy<ros_controllers_cartesian::PoseCommandInterface>::getState() const
{
  geometry_msgs::Pose pose = handle_.getPose();
  geometry_msgs::Twist twist = handle_.getTwist();
  geometry_msgs::Accel accel = handle_.getAccel();

  CartesianState state;
  state.p.x() = pose.position.x;
  state.p.y() = pose.position.y;
  state.p.z() = pose.position.z;
  state.q.x() = pose.orientation.x;
  state.q.y() = pose.orientation.y;
  state.q.z() = pose.orientation.z;
  state.q.w() = pose.orientation.w;

  state.v.x() = twist.linear.x;
  state.v.y() = twist.linear.y;
  state.v.z() = twist.linear.z;
  state.w.x() = twist.angular.x;
  state.w.y() = twist.angular.y;
  state.w.z() = twist.angular.z;

  state.v_dot.x() = accel.linear.x;
  state.v_dot.y() = accel.linear.y;
  state.v_dot.z() = accel.linear.z;
  state.w_dot.x() = accel.angular.x;
  state.w_dot.y() = accel.angular.y;
  state.w_dot.z() = accel.angular.z;

  return state;
}

//--------------------------------------------------------------------------------
// Joint position
//--------------------------------------------------------------------------------

bool ControlPolicy<hardware_interface::PositionJointInterface>::init(hardware_interface::RobotHW* hw,
                                                                     ros::NodeHandle& root_nh,
                                                                     ros::NodeHandle& controller_nh)
{
  if (!Base::init(hw, root_nh, controller_nh))
  {
    return false;
  };

  // Load user specified inverse kinematics solver
  std::string solver_type = controller_nh.param("ik_solver", std::string("example_solver"));

  solver_loader_ = std::make_unique<pluginlib::ClassLoader<IKSolver>>("cartesian_trajectory_controller", "ros_"
                                                                                                         "controllers_"
                                                                                                         "cartesian::"
                                                                                                         "IKSolver");
  try
  {
    ik_solver_.reset(solver_loader_->createUnmanagedInstance(solver_type));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return false;
  }

  if (!ik_solver_->init(robot_chain_, root_nh, controller_nh))
  {
    return false;
  }

  return true;
}

void ControlPolicy<hardware_interface::PositionJointInterface>::updateCommand(const CartesianState& cmd)
{
  const size_t size = joint_handles_.size();

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
    current(i) = joint_handles_[i].getPosition();
  }

  // Compute inverse kinematics
  ik_solver_->cartToJnt(current, goal, target);

  // Command each joint
  for (size_t i = 0; i < size; ++i)
  {
    joint_handles_[i].setCommand(target(i));
  }
}

//--------------------------------------------------------------------------------
// Joint velocity
//--------------------------------------------------------------------------------

bool ControlPolicy<hardware_interface::VelocityJointInterface>::init(hardware_interface::RobotHW* hw,
                                                                     ros::NodeHandle& root_nh,
                                                                     ros::NodeHandle& controller_nh)
{
  if (!Base::init(hw, root_nh, controller_nh))
  {
    return false;
  };

  ik_solver_ = std::make_unique<KDL::ChainIkSolverVel_wdls>(robot_chain_);
  return true;
}

void ControlPolicy<hardware_interface::VelocityJointInterface>::updateCommand(const CartesianState& cmd)
{
  const size_t size = joint_handles_.size();

  KDL::JntArray current(size);
  KDL::JntArray target(size);
  KDL::Twist goal;

  goal.vel[0] = cmd.v.x();
  goal.vel[1] = cmd.v.y();
  goal.vel[2] = cmd.v.z();
  goal.rot[0] = cmd.w.x();
  goal.rot[1] = cmd.w.y();
  goal.rot[2] = cmd.w.z();

  // Start where we are
  for (size_t i = 0; i < size; ++i)
  {
    current(i) = joint_handles_[i].getPosition();
  }

  // Compute joint velocities
  ik_solver_->CartToJnt(current, goal, target);

  // Command each joint
  for (size_t i = 0; i < size; ++i)
  {
    joint_handles_[i].setCommand(target(i));
  }
}

//--------------------------------------------------------------------------------
// Trajectory publishing
//--------------------------------------------------------------------------------

bool ControlPolicy<hardware_interface::JointStateInterface>::init(hardware_interface::RobotHW* hw,
                                                                  ros::NodeHandle& root_nh,
                                                                  ros::NodeHandle& controller_nh)
{
  if (!Base::init(hw, root_nh, controller_nh))
  {
    return false;
  };

  // Publishers
  pose_publisher_ = controller_nh.advertise<geometry_msgs::PoseStamped>("reference_pose", 10);
  twist_publisher_ = controller_nh.advertise<geometry_msgs::TwistStamped>("reference_twist", 10);

  return true;
}

void ControlPolicy<hardware_interface::JointStateInterface>::updateCommand(const CartesianState& cmd)
{
  // Pose
  geometry_msgs::PoseStamped p;
  p.header.frame_id = this->robot_base_;
  p.header.stamp = ros::Time::now();
  p.pose.position.x = cmd.p.x();
  p.pose.position.y = cmd.p.y();
  p.pose.position.z = cmd.p.z();
  p.pose.orientation.x = cmd.q.x();
  p.pose.orientation.y = cmd.q.y();
  p.pose.orientation.z = cmd.q.z();
  p.pose.orientation.w = cmd.q.w();

  pose_publisher_.publish(p);

  // Twist
  geometry_msgs::TwistStamped t;
  t.header.frame_id = this->robot_base_;
  t.header.stamp = ros::Time::now();
  t.twist.linear.x = cmd.v.x();
  t.twist.linear.y = cmd.v.y();
  t.twist.linear.z = cmd.v.z();
  t.twist.angular.x = cmd.w.x();
  t.twist.angular.y = cmd.w.y();
  t.twist.angular.z = cmd.w.z();

  twist_publisher_.publish(t);
}
}  // namespace ros_controllers_cartesian
