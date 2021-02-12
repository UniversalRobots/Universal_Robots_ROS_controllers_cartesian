# Cartesian Trajectory Controller
A simple Cartesian trajectory controller that uses the new Cartesian trajectory definition.

## Rationale
The action interface for Cartesian trajectories uses established ROS message
types. The new features currently enable two possible ways of execution:
Using the new interfaces and using current ROS control mechanisms. The table below highlights possible
applications.

| Hardware interface | Primary application |
| -------- | -------- |
| ``PoseCommandInterface``    | You want spline interpolation in ROS but the OEMs driver to take care of inverse kinematics (IK).
| ``PositionJointInterface``     | You want spline interpolation in ROS and implement your own IK solver. The provided example uses the established Weighted Levenberg-Marquardt solver form KDL.


## Controller .yaml
An example config for the Universal Robots UR10 looks like this:
```yaml
pose_cartesian_traj_controller:
    type: "pose_controllers/CartesianTrajectoryController"

    # UR driver convention
    base: "base"
    tip: "tool0_controller"

    joints:
       - shoulder_pan_joint
       - shoulder_lift_joint
       - elbow_joint
       - wrist_1_joint
       - wrist_2_joint
       - wrist_3_joint

jnt_cartesian_traj_controller:
    type: "position_controllers/CartesianTrajectoryController"

    # UR driver convention
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
