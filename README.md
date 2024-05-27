# MOCAP4ROS2 

[![GitHub Action
Status](https://github.com/MOCAP4ROS2-Project/mocap/workflows/rolling/badge.svg)](https://github.com/MOCAP4ROS2-Project/mocap)
[![codecov](https://codecov.io/gh/MOCAP4ROS2-Project/mocap/rolling/graph/badge.svg)](https://codecov.io/gh/MOCAP4ROS2-Project/mocap)

This project provides support for ROS2 integration with Vicon cameras (MOCAP systems based on vision) and Technaid TechMCS IMUs (MOCAP systems based on motion sensors).

The project [MOCAP4ROS2](https://rosin-project.eu/ftp/mocap4ros2) is funded as a Focused Technical Project by [ROSIN](http://rosin-project.eu/).


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

***

<p align="center"> 
<img align="center" src="https://github.com/MOCAP4ROS2-Project/mocap4ros2_exp_and_resources/blob/rolling/resources/mocap4ros_arch.png" 
    alt="mocap4ros_arch" width="100%">
</p>


***
## Description

This fork allows to read data from Vicon tracker and publish a ROS2 odometry message for a rigid body (atm only a single rigid body is supported). Body poses comes directly from Vicon readings, while linear and angular velocities are computed from differentiation of positions and orientations between 2 timestamps.
For additional info about angluar velocities computation, see: https://mariogc.com/post/angular-velocity-quaternions/.


## Usage

To run the node, use the following command:

`ros2 launch mocap4r2_robot_gt mocap4r2_robot_gt.launch.py`

After construction, the Vicon odometry message is published under `/vicon/odom`, together with the additional frame `base_link_gt` which is the center of the Vicon rigid body.

Note that, with this node, no precise alignment or positioning of the robot is needed. Just place the robot within the cameras workplace and start any localization stuff. To align the computed odometry with the ground truth one obtained from Vicon, you can use the service `/mocap4r2_gt/set_get_origin` with zero arguments, to align `base_link_gt` frame with `odom` frame.



