# Pioneer Gazebo Simulation

This is project aims at creating a simulating environment for the Pioneer 2DX. It is a mobile robot consisting of 2 fixed actuated parallel wheels in the back and a passive castor wheel in the front.With degree of mobility (2,0), this mobile robot is the perfect environment for demonstrating several robotics problem in guidance, localisation, control and as well as testing potential solutions and evaluating their performances.

This repository first contains a gazebo plugin for controlling the robot. The plugin can interface with a ROS, as it listens to the velocity commands for the fixed wheels, and publishes the global position w.r.t. the world frame. (more advanced settings would have the plugin publishing sensor data on which a localisation algorithm will be applied to estimate the global position)

The plugin source code can be find in the ```plugin``` folder

![alt text](https://github.com/MahmoudElOmar/pioneer-gazebo/blob/main/pioneer_2dx.png)
