#pragma once

#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utility.hpp>
#include "sensor_msgs/Image.h"
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cstdio>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

#include <iostream>
#include <opencv2/core/core.hpp>
#include <string>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <rosgraph_msgs/Clock.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/UInt64.h"
#include <time.h>
#include <image_transport/image_transport.h>

#include <dynamic_reconfigure/server.h>

#include "agribot_vs.h"

using namespace cv;
using namespace std;
using namespace Eigen;

namespace agribot_vs {

class AgribotVSNodeHandler {
 public:

  AgribotVS AgriBotVS;

  AgribotVSNodeHandler(ros::NodeHandle& node_handler);
  virtual ~AgribotVSNodeHandler();
  void CropRow_Tracking(camera& src);
  void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void image_front_calllback(const sensor_msgs::ImageConstPtr& msg);
  void image_back_calllback(const sensor_msgs::ImageConstPtr& msg);
  void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void Amcl_PoseCallback(const geometry_msgs::PoseStamped& msg);
  void StopForSec(float delay);
  void PublishVelocity(int _in=1);
  void TurnInPlace(float angle);

  ros::Publisher Time_pub;

 private:

  int state, in_state;

  // ROS node handle.
  ros::NodeHandle nodeHandle_;

  ros::Subscriber image_front_sub;
  ros::Subscriber image_back_sub;
  ros::Subscriber Mocap_sub;
  ros::Subscriber Odom_sub;
  ros::Subscriber IMU_sub;


  int _cnt;
  double mocap_roll, mocap_pitch, mocap_yaw;
  double imu_roll, imu_pitch, imu_yaw;
  ros::Publisher Log_pub;
  ros::Publisher VSVelocityPub;
  

};
}  // namespace agribot_vs
