## Visual-Servoing based Navigation for Monitoring Row-Crop Fields
---
<div align="center">
	<img src="/img/vs_poster.png" alt="visualservoing" width="850" title="visualservoing"/>
</div>

In this project, we propose a framework tailored for
navigation in row-crop fields by exploiting the regular crop-row structure present
in the fields. Our approach uses only the images from on-board cameras without
the need for performing explicit localization or maintaining a map of the field.
It allows the robot to follow the crop-rows accurately and handles the switch to
the next row seamlessly within the same framework. We implemented our approach
using C++ and ROS and thoroughly tested it in several simulated environments with different
shapes and sizes of field

to know more about its performance, please watch  the Youtube clip:([Click on the Image](https://youtu.be/xHxL8koNkfc))

[![AgriBot-Visual-Servoing based Navigation for Monitoring Row-Crop Fields](http://img.youtube.com/vi/Vy0LuEckN2Q/0.jpg)](https://youtu.be/Vy0LuEckN2Q "Visual-Servoing based Navigation for Monitoring Row-Crop Fields")


## Features
 - Capable of autonomously navigating mobile robots through the row-crop fields.  
 - Executable on embedded controllers with limit processing power (Odroid, Raspberry Pi).
 - Compatible with ROS. 
 - Capable of having same performance on different mobile robot platforms.
 - Providing simulation environment to ease testing process.

 <div align="center">
	<img src="/img/vs_graph.png" alt="visualservoing" width="400" title="visualservoing"/>
    <img src="/img/vs_em.png" alt="visualservoing" width="400" title="visualservoing"/>
</div>

## Dependencies
* c++11
* catkin
* opencv >= 2.4
* Eigen >= 3.3

Also a complete simulation package is provieded in [AgriBot Repository]() from [IPB](http://www.ipb.uni-bonn.de/) including  simualted row-crop fields and robot needed to test the application.

<div align="center">
	<img src="/img/motivation.png" alt="visualservoing" width="400" title="visualservoing"/>
    <img src="/img/motivation_old.png" alt="visualservoing" width="400" title="visualservoing"/>
</div>


## Build and Run *agribot_visualservoing* package
To launch the *agribot_visualservoing* simply follow these steps:
1. clone the package into your catkin_ws,
```bash
cd ~/catkin_ws/src
git clone https://github.com/PRBonn/visual-crop-row-navigation.git
```
2. make sure dependencies are provided,
3. build the package,
```bash
cd ~/catkin_ws
catkin_make or catkin build
```
4. you need to have two cameras with these topics already running:
```
* /front/rgb/image_raw [image]
* /back/rgb/image_raw [image]
```
to stream image from webcams you can use [usb_cam](http://wiki.ros.org/usb_cam)

5. launch package using:
```bash
$ roslaunch agribot_visualservoing visualservoing.launch
```
---
**Node Properties**
```
Node: [/agribot_vs]

Publications: 
 * /cmd_vel [geometry_msgs/Twist]
 * /vs_image [image]
 * /vs_msg [agribot_visualservoing/vs_msg]

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


## Download Test Bagfile

You can have a small bagfile containing required images from a filed to run the code easily.

Download from: 

---
## License

This project is licensed under the FreeBSD License. See the LICENSE.txt file for details.
