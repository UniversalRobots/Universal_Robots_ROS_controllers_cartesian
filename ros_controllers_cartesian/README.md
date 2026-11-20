This package brings mechanisms for Cartesian control to the ROS-control
framework.

## Overview

New functionality (colored):

![Colored: New contributions from this package or related\|width=100%](https://github.com/UniversalRobots/Universal_Robots_ROS_controllers_cartesian/blob/main/ros_controllers_cartesian/doc/cartesian_ros_control.png?raw=true)

## Rationale

As opposed to joint-based control, Cartesian control is often more
intuitive for programmers to specify how a tool (robot end-effector)
should move in their application. For instance, gluing, grinding,
polishing and all sorts of other surface-related tasks benefit from a
straight-forward task formulation with Cartesian coordinates.

Currently, there is no support of Cartesian control in ROS. Yet, the
number of OEMs, whose drivers support Cartesian control interfaces is
growing. This set of packages aims at filling this gap and get you
started with Cartesian control.

## Major features at a glance

- **Add Cartesian functionality to ROS control**. This brings new
  interfaces for controller design, such as a
  ```PoseCommandInterface```, a ```TwistCommandInterface```,
  and a new Cartesian trajectory definition. Example controllers include
  a ```TwistController``` and a
  ```CartesianTrajectoryController```.

<!-- -->

- **Enable Cartesian trajectory control** in your applications. Specify
  your task comfortably with waypoints in task space. ROS-side
  interpolation and streaming of setpoints over the new interfaces is
  only one of several alternatives. See the
  [`cartesian_control_msgs`](https://github.com/UniversalRobots/Universal_Robots_ROS_cartesian_control_msgs)
  package for details about the interface definition.

<!-- -->

- **Use (conventional) ROS control** for Cartesian trajectory execution.
  You don't need to change anything in the driver's HW-abstraction of
  your specific robot if that supports current ROS control. The
  `position_controllers/CartesianTrajectoryController` controller uses
  an IK solver to generate joint commands from the Cartesian trajectory
  during runtime. Thus, existing joint command interfaces (which should
  be present in most ROS-Control-enabled robot drivers) are sufficient
  for using this controller.

<!-- -->

- **Hand-over control to the robot by forwarding trajectories**. Two new
  interfaces ```CartesianTrajectoryInterface``` and
  ```JointTrajectoryInterface``` let robots take care of
  driver-side interpolation to achieve best performance. This
  functionality lives in a
  [separate repository](https://github.com/UniversalRobots/Universal_Robots_ROS_passthrough_controllers).

<!-- -->

- **Speed-scale trajectory execution**. All trajectory executions (both
  Cartesian and joint-based) can be speed-scaled within 0 to 100% at
  runtime. This gives you flexibility in setting-up new applications and
  during test runs. Changing this continuously even lets you reshape
  trajectory execution without re-teaching. This functionality lives in
  a
  [separate repository](https://github.com/UniversalRobots/Universal_Robots_ROS_scaled_controllers)
  but is used for controllers in this repository.
