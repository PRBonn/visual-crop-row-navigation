/**************************************************************************/
/* Paper: Visual-Servoing based Navigation for Monitoring Row-Crop Fields */
/*    Alireza Ahmadi, Lorenzo Nardi, Nived Chebrolu, Cyrill Stachniss     */
/*         All authors are with the University of Bonn, Germany           */
/* maintainer: Alireza Ahmadi                                             */
/*          (Alireza.Ahmadi@uni-bonn.de / http://alirezaahmadi.xyz)       */
/**************************************************************************/

#pragma once

#include "agribot_vs.h"
#include <rosgraph_msgs/Clock.h>

using namespace cv;
using namespace std;
using namespace Eigen;

namespace agribot_vs {

class AgribotVSNodeHandler {
 public:

  AgribotVSNodeHandler(ros::NodeHandle& node_handler);
  virtual ~AgribotVSNodeHandler();
  void CropRow_Tracking(camera& src);
  void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void image_front_calllback(const sensor_msgs::ImageConstPtr& msg);
  void image_back_calllback(const sensor_msgs::ImageConstPtr& msg);
  void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void Amcl_PoseCallback(const geometry_msgs::PoseStamped& msg);
  void StopForSec(float delay);
  void publishVelocity(int _in=1);
  // void dynamicReconfig_callback(agribot_visualservoing::AgribotVSConfig &config, uint32_t level);

  ros::Publisher Time_pub;
  AgribotVS agribotVS;

 private:

  int state, in_state;

  // ROS node handle.
  ros::NodeHandle nodeHandle_;

  ros::Subscriber image_front_sub;
  ros::Subscriber image_back_sub;
  ros::Subscriber Mocap_sub;
  ros::Subscriber Odom_sub;
  ros::Subscriber IMU_sub;
  
  double mocap_roll, mocap_pitch, mocap_yaw;
  double imu_roll, imu_pitch, imu_yaw;
  ros::Publisher Log_pub;
  ros::Publisher VSVelocityPub;
  

};
}  // namespace agribot_vs
