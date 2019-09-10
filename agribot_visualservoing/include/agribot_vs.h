/**************************************************************************/
/* Paper: Visual-Servoing based Navigation for Monitoring Row-Crop Fields */
/*    Alireza Ahmadi, Lorenzo Nardi, Nived Chebrolu, Cyrill Stachniss     */
/*         All authors are with the University of Bonn, Germany           */
/* maintainer: Alireza Ahmadi                                             */
/*          (Alireza.Ahmadi@uni-bonn.de / http://alirezaahmadi.xyz)       */
/**************************************************************************/

#pragma once

#include "ros/ros.h"

#include <cstdio>
#include <cstdlib>
#include <vector>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm> 
#include <iterator>  

#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/opencv.hpp>

#include "sensor_msgs/Image.h"
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR> 
#include <Eigen/Geometry>

#include <dynamic_reconfigure/server.h>

#include "agribot_visualservoing/vs_msg.h"
#include "agribot_types.h"

#define DEG2RAD 0.0174533
#define RAD2DEG 57.2958

#define sind(x) (sin(fmod((x),360) * M_PI / 180))
#define cosd(x) (cos(fmod((x),360) * M_PI / 180))

using namespace cv;
using namespace std;
using namespace Eigen;

namespace agribot_vs {

class AgribotVS {
 public:
  AgribotVS();
  ~AgribotVS();

  bool readRUNParmas(ros::NodeHandle& nodeHandle_);

  vector<vector<Point>> CropRowFeatures(Mat& img);
  vector<Vec4i> EdgeDetector(Mat& Input_Image, float Scale);
  vector<Vec4i> FitLineOnContures(Mat& img, vector<Point2f>& ContourCenters);
  vector<Point2f> getContureCenters(Mat& img, vector<vector<Point>>& contours);
  vector<Point2f> filterContures(Mat& img, vector<vector<Point>>& contours);

  void compute_feature_point(camera& I);
  void draw_features(camera& I, Vector3f Feature, cv::Scalar color);

  void Controller(camera& I_primary, camera& I_secondary);
  void switch_cameras(int& cam_primary);
  void switching_controller(camera& I_primary, camera& I_secondary, unsigned int min_points);

  int compute_intersection(Point2f& P, Point2f& Q);
  void compute_intersection_old(Point2f& P, Point2f& Q);

  MatrixXf is_in_image_point(MatrixXf R);

  float compute_Theta(Point2f& P, Point2f& Q);
  Vector2f dist(MatrixXf& A, MatrixXf& B);
  float wrapToPi(float angle);
  Vector2f hom2euc(Vector3f Mat);
  VectorXi find(Eigen::Vector4i A);

  void shift_neighbourhood(camera& I, int shift_dir=1);
  void initialize_neigbourhood(camera& I);
  void is_in_neigbourhood(camera& I);
  void draw_neighbourhood(camera& I);
  vector<Point2f> Compute_nh_ex(camera& I);
  
  Vector4i is_in_image(MatrixXf R);

  Point2f camera2image(Point2f& xc);
  Point2f camera2origin(Point2f& xc);

  Point2f origin2image(Point2f& xc);
  Point2f origin2camera(Point2f& xc);
  
  Point2f image2camera(Point2f& xc);
  Point2f image2origin(Point2f& xc);

  std::vector<double> getEulerAngles(const nav_msgs::Odometry::ConstPtr& Pose);

  // This file defines the controller parameters
  camera front_cam;
  camera back_cam;

  int cnt_off;

  agribot_visualservoing::vs_msg VSMsg;

  double rho_b;
  double rho_f;
  double rho;
  double ty;
  double tz;
  double vf_des;
  double vb_des;
  double del_t;

  double coef;
  int min_frame;

  int nh_L;
  int nh_H;
  
  int nh_offset;

  int mode;
  int w_control_mode;

  double w_max;
  double w_min;
  double z_min;

  int ex_Xc, ex_Yc;

  double lambda_x_1, lambda_x_2 ,lambda_x_3, lambda_x_4;
  double lambda_w_1, lambda_w_2 ,lambda_w_3, lambda_w_4;

  double div_soft;

  int steering_dir;
  bool drive_forward;
  int driving_dir;
  bool turning_mode;
  int height;
  int width;

  int navigation_dir;     // Main navigation direction 1 = -> (L to R) , -1 = <- (R to L)
  int min_points_switch; // Minimum number of points to consider changing camera

  bool switching_controls_drive_forward;
  bool switching_controls_turning_mode;
  int switching_controls_steering_dir;
  int cam_num;

  int minp_cnt;

  int id;
  Point2f P;
  Point2f Q;
  Vector3f F;
  Vector3f F_des;
  int side;

  bool publish_cmd_vel,publish_linear_vel;

  vector<Vec4i> CenterLine;
  geometry_msgs::Twist VelocityMsg;
  sensor_msgs::Imu imu;
  vector<double> RobotPose;
  vector<double> RobotLinearVelocities;
  vector<double> RobotAngularVelocities;
  vector<double> RotationVec;
  vector<double> TransVec;

  std::vector<Vec4i> Vectlines;

  bool mask_tune;
  bool single_camera_mode;
  int FilterQuieSize;

  double Scale;

  float x,y;

  double LineDiffOffset;

  std::vector<double> x_poses;
  std::vector<double> y_poses;
  std::vector<double> z_poses;

  Mat img_contour;

  int max_Hue;
  int min_Hue;
  int max_Saturation;
  int min_Saturation;
  int max_Value;
  int min_Value;

  double minContourSize;

  int center_min;
  int center_max;
  int center_min_off;
  int center_max_off;

  double max_vel_lin_;
  double min_vel_lin_;
  double max_incr_lin_;
  double k_p_lin_;
  double k_i_lin_;
  double k_d_lin_;

  double max_vel_ang_;
  double min_vel_ang_;
  double max_incr_ang_;
  double k_p_ang_;
  double k_i_ang_;
  double k_d_ang_;

  int LineFitting_method;
  int ControllerType;

 private:

  #define d_t_ 1 / 20;
  double error_ang_, integral_ang_;
  double error_lin_, integral_lin_;
  
};
}  // namespace agribot_vs
