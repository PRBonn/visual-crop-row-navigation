/***************************************************************************************/
/* Paper: Visual-Servoing based Navigation for Monitoring Row-Crop Fields              */
/*    Alireza Ahmadi, Lorenzo Nardi, Nived Chebrolu, Chis McCool, Cyrill Stachniss     */
/*         All authors are with the University of Bonn, Germany                        */
/* maintainer: Alireza Ahmadi                                                          */
/*          (Alireza.Ahmadi@uni-bonn.de / http://alirezaahmadi.xyz)                    */
/***************************************************************************************/

#pragma once

#include "agribot_vs.h"
#include <rosgraph_msgs/Clock.h>

using namespace cv;
using namespace std;
using namespace Eigen;

namespace agribot_vs {
/**
 * @brief node handler class of VisualServoing application
 * 
 */
class AgribotVSNodeHandler {
 public:
  /**
   * @brief Construct a new Agribot V S Node Handler object
   * 
   * @param node_handler 
   */
  AgribotVSNodeHandler(ros::NodeHandle& node_handler);
  /**
   * @brief Destroy the Agribot V S Node Handler object
   * 
   */
  virtual ~AgribotVSNodeHandler();
  /**
   * @brief gets the input camera (primary one) to extract crop-rows and visual featues
   * 
   * @param src is primary camera
   */
  void CropRow_Tracking(camera& src);

  void imuCallBack(const sensor_msgs::Imu::ConstPtr& msg);
  /**
   * @brief gets front camera's image
   * 
   * @param msg 
   */
  void imageFrontCalllBack(const sensor_msgs::ImageConstPtr& msg);
  /**
   * @brief gets rear camera's image
   * 
   * @param msg 
   */
  void imageBackCalllBack(const sensor_msgs::ImageConstPtr& msg);
  /**
   * @brief gets the robot odometry from base controller
   * 
   * @param msg 
   */
  void odomCallBack(const nav_msgs::Odometry::ConstPtr& msg);
  /**
   * @brief gets the poseof the robot in Lab from Mocap system
   * 
   * @param msg 
   */
  void amclPoseCallBack(const geometry_msgs::PoseStamped& msg);
  /**
   * @brief stops the robot fror given time
   * 
   * @param delay 
   */
  void StopForSec(float delay);
  /**
   * @brief publishes /cmd_vel topic to move the robot 
   * 
   * @param _in 
   */
  void publishVelocity(int _in=1);
  // void dynamicReconfig_callback(visual_crop_row_navigation::AgribotVSConfig &config, uint32_t level);

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
