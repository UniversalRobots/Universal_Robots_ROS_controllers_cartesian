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
/*!\file    control_policies.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2021/01/24
 *
 */
//-----------------------------------------------------------------------------

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <speed_scaling_interface/speed_scaling_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <cartesian_interface/cartesian_command_interface.h>
#include <cartesian_trajectory_interpolation/cartesian_trajectory.h>
#include "cartesian_trajectory_interpolation/cartesian_trajectory_segment.h"
#include "hardware_interface/joint_state_interface.h"
#include "kdl/chainfksolver.hpp"
#include <cartesian_trajectory_interpolation/cartesian_state.h>
#include <pluginlib/class_loader.h>
#include <inverse_kinematics/ik_solver_base.h>

// KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <memory>

namespace ros_controllers_cartesian
{
/**
 * @brief A common control type with optional speed scaling interface
 *
 * @tparam HWInterface Hardware interface essential for control
 */
template <class HWInterface>
using Controller =
    controller_interface::MultiInterfaceController<HWInterface, scaled_controllers::SpeedScalingInterface>;

/**
 * @brief A common base class for joint-based control policies
 *
 * @tparam HWInterface The hardware interface for joint actuation
 * @tparam HandleType The type of joint handles to use.
 */
template <class HWInterface, class HandleType>
class JointBasedController : public Controller<HWInterface>
{
public:
  JointBasedController() : Controller<HWInterface>(true){};  // optional speedscaling

  virtual bool init(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  CartesianState getState() const;

protected:
  std::vector<HandleType> joint_handles_;
  std::unique_ptr<KDL::ChainFkSolverVel_recursive> fk_solver_;
  KDL::Chain robot_chain_;
  std::string robot_base_;
  std::string robot_tip_;
};

/**
 * @brief Primary template
 *
 * Fail compilation for all non-specialized control policies.
 *
 * @tparam HWInterface Hardware interface essential for control.
 */
template <class HWInterface>
class ControlPolicy;

/**
 * @brief Specialization for Cartesian pose control
 */
template <>
class ControlPolicy<ros_controllers_cartesian::PoseCommandInterface>
  : public Controller<ros_controllers_cartesian::PoseCommandInterface>
{
public:
  ControlPolicy() : Controller<ros_controllers_cartesian::PoseCommandInterface>(true){};  // optional speedscaling

  bool init(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  /**
   * @brief Set the HW interface's command buffer
   *
   * @param cmd Desired Cartesian state of the manipulator
   */
  void updateCommand(const CartesianState& cmd);

  CartesianState getState() const;

private:
  std::string base_;
  std::string tip_;
  ros_controllers_cartesian::PoseCommandHandle handle_;
};

/**
 * @brief Specialization for Cartesian twist control
 */
template <>
class ControlPolicy<ros_controllers_cartesian::TwistCommandInterface>
  : public Controller<ros_controllers_cartesian::TwistCommandInterface>
{
public:
  ControlPolicy()
    : Controller<ros_controllers_cartesian::TwistCommandInterface>(true)  // optional speedscaling
    {};

  /**
   * @brief Set the HW interface's command buffer
   *
   * @param cmd Desired Cartesian state of the manipulator
   */
  void updateCommand(const ros_controllers_cartesian::CartesianState& cmd);

  ros_controllers_cartesian::CartesianState getState() const;
};

/**
 * @brief Specialization for joint position control
 */
template <>
class ControlPolicy<hardware_interface::PositionJointInterface>
  : public JointBasedController<hardware_interface::PositionJointInterface, hardware_interface::JointHandle>
{
public:
  using Base = JointBasedController<hardware_interface::PositionJointInterface, hardware_interface::JointHandle>;

  bool init(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  /**
   * @brief Set the HW interface's command buffer
   *
   * Uses KDL's Levenberg-Marquardt Inverse Kinematics solver for mapping
   * poses to joint positions.
   *
   * @param cmd Desired Cartesian state of the manipulator
   */
  void updateCommand(const CartesianState& cmd);

private:
  std::unique_ptr<pluginlib::ClassLoader<IKSolver> > solver_loader_;
  std::unique_ptr<IKSolver> ik_solver_;
};

/**
 * @brief Specialization for joint velocity control
 */
template <>
class ControlPolicy<hardware_interface::VelocityJointInterface>
  : public JointBasedController<hardware_interface::VelocityJointInterface, hardware_interface::JointHandle>
{
public:
  using Base = JointBasedController<hardware_interface::VelocityJointInterface, hardware_interface::JointHandle>;

  bool init(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  /**
   * @brief Set the HW interface's command buffer
   *
   * Uses KDL's weighted DLS method for mapping twists to joint velocitiies.
   *
   * @param cmd Desired Cartesian state of the manipulator
   */
  void updateCommand(const CartesianState& cmd);

private:
  std::unique_ptr<KDL::ChainIkSolverVel_wdls> ik_solver_;
};

/**
 * @brief Specialization for a read-only controller for trajectory publishing
 */
template <>
class ControlPolicy<hardware_interface::JointStateInterface>
  : public JointBasedController<hardware_interface::JointStateInterface, hardware_interface::JointStateHandle>
{
public:
  using Base = JointBasedController<hardware_interface::JointStateInterface, hardware_interface::JointStateHandle>;

  bool init(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  /**
   * @brief Publish Cartesian state to pose and twist topics
   *
   * @param cmd Desired Cartesian state of the manipulator
   */
  void updateCommand(const CartesianState& cmd);

private:
  ros::Publisher pose_publisher_;
  ros::Publisher twist_publisher_;
};

}  // namespace ros_controllers_cartesian

#include <cartesian_trajectory_controller/control_policies.hpp>
