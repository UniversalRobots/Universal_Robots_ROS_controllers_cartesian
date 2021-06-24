# Cartesian Interface

This package provides new hardware interfaces to implement Cartesian ROS control
for robot manipulators.

## Rationale
Several OEMs provide native Cartesian interfaces in their robot drivers.
By offering new according hardware interfaces, implementers of ROS controllers get direct access to Cartesian buffers for reading and writing.
Whether this provides advantages over the classic joint-based interfaces of course depends on personal use cases.
The main difference is that in the Cartesian case the robot takes care of Inverse Kinematics.

## Usage
This package provides ``PoseCommandInterface`` and ``TwistCommandInterface``
for read/write access and ``CartesianStateInterface`` for pure read access.
You would add them to your hardware abstraction like this:

```c++
#include <cartesian_interface/cartesian_command_interface.h>
#include <cartesian_interface/cartesian_state_handle.h>

class YourRobot : public hardware_interface::RobotHW
{
...

  // New interfaces for Cartesian ROS-controllers
  ros_controllers_cartesian::CartesianStateInterface cart_interface_;
  ros_controllers_cartesian::TwistCommandInterface twist_interface_;
  ros_controllers_cartesian::PoseCommandInterface pose_interface_;
  ...

  // Buffers for read/write access
  geometry_msgs::Pose cart_pose_;
  geometry_msgs::Twist cart_twist_;
  geometry_msgs::Accel cart_accel_;
  geometry_msgs::Accel cart_jerk_;
  geometry_msgs::Twist twist_command_;
  geometry_msgs::Pose pose_command_;
  ...

}
```
Registering them could look like this:
```c++
YourRobot::YourRobot()
{
...
  ros_controllers_cartesian::CartesianStateHandle cart_state_handle("base", "tip", &cart_pose_, &cart_twist_,
                                                                &cart_accel_, &cart_jerk_);
  cart_interface_.registerHandle(cart_state_handle);

  twist_interface_.registerHandle(
      ros_controllers_cartesian::TwistCommandHandle(cart_interface_.getHandle("tip"), &twist_command_));
  twist_interface_.getHandle("tip");

  pose_interface_.registerHandle(
      ros_controllers_cartesian::PoseCommandHandle(cart_interface_.getHandle("tip"), &pose_command_));
  pose_interface_.getHandle("tip");

  // Register interfaces
  registerInterface(&twist_interface_);
  registerInterface(&pose_interface_);
  ...
}

```

Note the two strings `base` and `tip` during instantiation of the Cartesian state handle.
They represent frames in your robot kinematics and their names will vary from robot to robot.
There's a convention behind that assumes that `base` is the reference frame in which `tip` is given.
`tip` is the end-point of the robot we want to control and represents a unique identifier for the handle.
You can think of it as resource names in joint-based ROS control.


## Cartesian ROS controllers

When implementing a ROS controller, you would add the according handle for
read/write access, e.g. in the case of Cartesian pose control:
```c++
#include <cartesian_interface/cartesian_command_interface.h>
...

class YourController : public controller_interface::Controller<ros_controllers_cartesian::PoseCommandInterface>
{

...
private:
  ros_controllers_cartesian::PoseCommandHandle handle_;
...
}
```
During instantiation of the handle, you ask for the end-point you want to control:
```c++
  handle_ = hw->getHandle("tip");
```
and then use that handle to implement your control loop in `update(...)`:
```c++

  // Get current state from the robot hardware
  geometry_msgs::Pose pose = handle_.getPose();
  geometry_msgs::Twist twist = handle_.getTwist();
  geometry_msgs::Accel accel = handle_.getAccel();

  geometry_msgs::Pose target;

  // Implement your control law here

  handle_.setCommand(target);
...
```

## Acknowledgement
Developed in collaboration between:

[<img height="60" alt="Universal Robots A/S" src="../ros_controllers_cartesian/doc/resources/ur_logo.jpg">](https://www.universal-robots.com/) &nbsp; and &nbsp;
[<img height="60" alt="FZI Research Center for Information Technology" src="../ros_controllers_cartesian/doc/resources/fzi_logo.png">](https://www.fzi.de).

***
<!--
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png"
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg"
     alt="eu_flag" height="45" align="left" >

This project has received funding from the European Union’s Horizon 2020
research and innovation programme under grant agreement no. 732287.
