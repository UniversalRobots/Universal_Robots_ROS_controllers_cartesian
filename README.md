# Cartesian Trajectory Controller
A simple Cartesian trajectory controller that uses the new Cartesian trajectory definition.

## Rationale
This controller shall get you started with executing Cartesian trajectories on your robot.
It implements a simple action server for `FollowCartesianTrajectory`. There are currently two ways of execution, depending on the interface provided by the robot.

The table below highlights possible applications.

| Hardware interface | Primary application |
| -------- | -------- |
| ``PoseCommandInterface``    | You want spline interpolation in ROS but the OEMs driver to take care of inverse kinematics (IK). This variant requires the new Cartesian interfaces.
| ``PositionJointInterface``     | You want spline interpolation in ROS and implement your own IK solver. The provided example uses the established Weighted Levenberg-Marquardt solver form KDL. This variant is compatible with current ROS-control so that you need not change your RobotHW abstraction.


## Controller .yaml
An example config for the Universal Robots UR10 looks like this:
```yaml
# This controller uses a Cartesian pose interface for control.
# Inverse Kinematics is handled by the robot itself.
pose_cartesian_traj_controller:
    type: "pose_controllers/CartesianTrajectoryController"

    # This type uses the names according to UR driver convention.
    # They need not neccessarily exist in the robot_description
    # and are only used to get the correct control handle from the
    # hardware interface.
    base: "base"
    tip: "tool0_controller"

    joints:
       - shoulder_pan_joint
       - shoulder_lift_joint
       - elbow_joint
       - wrist_1_joint
       - wrist_2_joint
       - wrist_3_joint

# This controller uses a joint position interface for control.
# Inverse Kinematics is handled by the controller.
jnt_cartesian_traj_controller:
    type: "position_controllers/CartesianTrajectoryController"

    # This type requires valid URDF links and they are used to
    # set-up an internal kinematics chain. Make sure that they correspond to the drivers' frames.
    # In this case, tool0 and tool0_controller are identical, but the latter
    # one does not exist in the URDF description.
    base: "base"
    tip: "tool0"

    joints:
       - shoulder_pan_joint
       - shoulder_lift_joint
       - elbow_joint
       - wrist_1_joint
       - wrist_2_joint
       - wrist_3_joint

```

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
