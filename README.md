# Visual Crop Row Navigation

<div align="center">
	<img src=".readme/vs_poster.png" alt="visual_servoing_husky" height="180" title="visual_servoing_husky"/>
</div>

This is a visual-servoing based robot navigation framework tailored for navigating in row-crop fields.
It uses the images from two on-board cameras and exploits the regular crop-row structure present in the fields for navigation, without performing explicit localization or mapping. It allows the robot to follow the crop-rows accurately and handles the switch to the next row seamlessly within the same framework.

This implementation uses C++ and ROS and has been tested in different environments both in simulation and in real world and on diverse robotic platforms.

This work has been developed @ [IPB](http://www.ipb.uni-bonn.de/), University of Bonn.

Check out the [video1](https://www.youtube.com/watch?v=uO6cgBqKBas), [video2](https://youtu.be/KkCVQAhzS4g) of our robot following this approach to navigate on a test row-crop field.

<div align="center">
	<a href="http://www.youtube.com/watch?feature=player_embedded&v=0qg6n4sshHk
		" target="_blank"><img src=".readme/husky_test.gif" alt="husky_navigation" height="250" title="husky_navigation" border="5"/><img src=".readme/husky_test_nav.gif" alt="husky_navigation" height="250" title="husky_navigation" border="5"/></a>
	<!-- <a href="http://www.youtube.com/watch?feature=player_embedded&v=0qg6n4sshHk
		" target="_blank"><img src="http://img.youtube.com/vi/0qg6n4sshHk/0.jpg"
		alt="Watch video" height="250" border="10" /></a> -->
</div>


## Features

- No maps or localization required.
- Running on embedded controllers with limit processing power (Odroid, Raspberry Pi).
- Simulation environment in Gazebo.
- Robot and cameras agnostic.

## Robotic setup

This navigation framework is designed for mobile robots equipped with two cameras mounted respectively looking to the front and to the back of the robot as illustrated in the picture below.

 <div align="center">
	<img src=".readme/vs_graph.png" alt="agribot_3d" height="250" title="agribot_3d"/>
    <img src=".readme/vs_em.png" alt="camera_img" height="250" title="camera_img"/>
</div>

A complete Gazebo simulation package is provided in [agribot_robot](https://github.com/PRBonn/agribot) repository including simulated row-crop fields and robot for testing the navigation framework.

<div align="center">
	<img src=".readme/motivation.png" alt="husky_navigation" height="280" title="husky_navigation"/>
    <img src=".readme/motivation_old.png" alt="gazebo_navigation" height="280"title="gazebo_navigation"/>
</div>

## Dependencies

- c++11
- catkin
- opencv >= 2.4
- Eigen >= 3.3

## How to build and run

1. Clone the package into your *catkin_ws*
```bash
cd ~/catkin_ws/src
git clone https://github.com/PRBonn/visual_crop_row_navigation.git
```
2. Build the package
```bash
cd ~/catkin_ws
catkin build visual_crop_row_navigation
```
3. Run ROS driver to stream images from the robot's cameras, for example using [usb_cam](http://wiki.ros.org/usb_cam)
<!-- ```
* /front/rgb/image_raw [image]
* /back/rgb/image_raw [image]
``` -->
4. Run visual servoing navigation
```bash
roslaunch visual_crop_row_navigation visualservoing.launch
```

<!-- 
**Node Properties**
```
Node: [/agribot_vs]

Publications:
 * /cmd_vel [geometry_msgs/Twist]
 * /vs_image [image]
 * /vs_msg [visual_crop_row_navigation/vs_msg]

Subscriptions:
 * /amcl_pose [pose]
 * /odom [geometry_msg/odometry]
 * /front/rgb/image_raw [image]
 * /back/rgb/image_raw [image]
 * /zed/camera/left/image_raw [sensor_msgs/Image]

Services:
 * None

**Parameters**
 * None
```
--- 
 -->

Successfully tested using:
- Ubuntu 16.04
- ROS kinetic

## Test data

Download the bagfile used for our experiments [here]().

## Simulation 

Simultion and robot packages can be found in [Agribot repo](https://github.com/PRBonn/agribot)


## Citation 
if you use this project in your recent works please refernce to it by:

```bash

@article{ahmadi2019visual,
  title={Visual Servoing-based Navigation for Monitoring Row-Crop Fields},
  author={Ahmadi, Alireza and Nardi, Lorenzo and Chebrolu, Nived and Stachniss, Cyrill},
  journal={arXiv preprint arXiv:1909.12754},
  year={2019}
}
```

## Acknowledgments
This work has partly been supported by the German Research Foundation under Germany’s Excellence Strategy, EXC-2070 - 390732324 ([PhenoRob](http://www.phenorob.de/)).
