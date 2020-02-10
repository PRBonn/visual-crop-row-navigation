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

#include "visual_crop_row_navigation/vs_msg.h"
#include "agribot_types.h"

#define DEG2RAD 0.0174533
#define RAD2DEG 57.2958

#define sind(x) (sin(fmod((x),360) * M_PI / 180))
#define cosd(x) (cos(fmod((x),360) * M_PI / 180))

using namespace cv;
using namespace std;
using namespace Eigen;

namespace agribot_vs {
/**
 * @brief main class which offers functions to perform visualServoing
 * 
 */
class AgribotVS {
 public:
  /**
   * @brief Construct a new Agribot V S:: Agribot V S object
   * 
   */
  AgribotVS();
  /**
   * @brief Destroy the Agribot V S:: Agribot V S object
   * 
   */
  ~AgribotVS();

  /**
   * @brief - function to load run params 
   * 
   * @param nodeHandle_ - ROS node handle
   * @return true - in case of loading all params
   * @return false - in case of failure
   */
  bool readRUNParmas(ros::NodeHandle& nodeHandle_);
  /**
   * @brief - detects features(contours in specific range of color) from inpout image
   * 
   * @param img - input - image of front or back cameras
   * @return vector<vector<Point>> - contours extracted from image 
   */
  vector<vector<Point>> CropRowFeatures(Mat& img);
  /**
   * @brief main functoin to control the robot baesd on extracted features
   * 
   * @param I_primary camera
   * @param I_secondary camera
   */
  void Controller(camera& I_primary, camera& I_secondary);
  /**
   * @brief using simple regression fits line on extracted features from image
   * 
   * @param img input image (draws the line on img)
   * @param ContourCenters 
   * @return vector<Vec4i> average line (the best fit)
   */
  vector<Vec4i> FitLineOnContures(Mat& img, vector<Point2f>& ContourCenters);
  /**
   * @brief estimates center of passed contours
   * 
   * @param img is input image, contours will be printed on top of it
   * @param contours - extracted features from image
   * @return vector<Point2f> - center of passed contours 
   */
  vector<Point2f> getContureCenters(Mat& img, vector<vector<Point>>& contours);
  /**
   * @brief fillters contours based on the fitting polygon/circle radius
   * 
   * @param img input image
   * @param contours extracted contours
   * @return vector<Point2f> flitered contours
   */
  vector<Point2f> filterContures(Mat& img, vector<vector<Point>>& contours);
  /**
   * @brief computes control feature points, visualized on the image corners
   * 
   * @param I  primary camera containing features
   */
  void compute_feature_point(camera& I);
  /**
   * @brief prints the features on the primary image
   * 
   * @param I primary camera
   * @param Feature extracted features inside the neighbourhood
   * @param color of visualiation
   */
  void draw_features(camera& I, Vector3f Feature, cv::Scalar color);
  /**
   * @brief switches primary and secondary cameras 
   * 
   * @param cam_primary id of current primary camera (front-> 1 back ->2)
   */
  void switch_cameras(int& cam_primary);
  /**
   * @brief handles switching operation on the senario of crop row following
   * 
   * @param I_primary camera
   * @param I_secondary camera
   * @param min_points minimum points inside the target image which wants to switch from
   */
  void switching_controller(camera& I_primary, camera& I_secondary, unsigned int min_points);
  /**
   * @brief computes intersection of line passing through P,Q
   * with four different edges of the image
   * 
   * @param P point in the bottom
   * @param Q point in the top
   * @return int returns index of the edge (top to bottom CW 0-3)
   */
  int compute_intersection(Point2f& P, Point2f& Q);
  /**
   * @brief checkes for coordinates of feature to be inside the image
   * 
   * @param R 
   * @return MatrixXf 
   */
  Vector4i is_in_image(MatrixXf R);
  /**
   * @brief checkes for coordinates of feature to be inside the neighbourhood
   * 
   * @param R 
   * @return MatrixXf 
   */
  MatrixXf is_in_image_point(MatrixXf R);
  /**
   * @brief computes angle difference between 
   * fitted line on P-Q and reference line on the middle
   * 
   * @param P point in the bottom
   * @param Q point in the top
   * @return float 
   */
  float compute_Theta(Point2f& P, Point2f& Q);
  /**
   * @brief computes distance between two pointsets
   * 
   * @param A 
   * @param B 
   * @return Vector2f distance of paires of A-B features
   */
  Vector2f dist(MatrixXf& A, MatrixXf& B);
  /**
   * @brief warps the input angle to Pi
   * 
   * @param angle input
   * @return float warped angle
   */
  float wrapToPi(float angle);
  /**
   * @brief converts back point from homogenous coordinate
   * system to Euclidian coordinate system
   * 
   * @param Mat 
   * @return Vector2f 
   */
  Vector2f hom2euc(Vector3f Mat);
  /**
   * @brief gets the ids of featues with non-zero value in A
   * 
   * @param A 
   * @return VectorXi 
   */
  VectorXi find(Eigen::Vector4i A);
  /**
   * @brief shifts neighbourhood windows in passed camera 
   * with input direction and valued laoded from params 
   * 
   * @param I passed camera
   * @param shift_dir direction of movement
   */
  void shift_neighbourhood(camera& I, int shift_dir=1);
  /**
   * @brief used to initialize the neighbourhood windon the center of image
   * 
   * @param I camera
   */
  void initialize_neigbourhood(camera& I);
  /**
   * @brief updates points layed in the camera's neigbhourhood 
   * 
   * @param I input camera
   */
  void is_in_neigbourhood(camera& I);
  /**
   * @brief prints neighbourhood on the image 
   * 
   * @param I input camera
   */
  void draw_neighbourhood(camera& I);
  /**
   * @brief 
   * 
   * @param I 
   * @return vector<Point2f> 
   */
  vector<Point2f> Compute_nh_ex(camera& I);
  /**
   * @brief projects point from camera to image coordinate system
   * 
   * @param xc  input image in camera coordinates
   * @return Point2f output image in image coordinate system
   */
  Point2f camera2image(Point2f& xc);
  /**
   * @brief projects point from camera to image coordinate system
   * 
   * @param xc  input image in camera coordinates
   * @return Point2f output image in origin coordinate system
   */
  Point2f camera2origin(Point2f& xc);
  /**
   * @brief projects point from origin to image coordinate system
   * 
   * @param xc  input image in origin coordinates
   * @return Point2f output image in image coordinate system
   */
  Point2f origin2image(Point2f& xc);
  /**
   * @brief projects point from origin to camera coordinate system
   * 
   * @param xc  input origin in image coordinates
   * @return Point2f output image in camera coordinate system
   */
  Point2f origin2camera(Point2f& xc);
  /**
   * @brief projects point from image to camera coordinate system
   * 
   * @param xc  input image in image coordinates
   * @return Point2f output image in camera coordinate system
   */
  Point2f image2camera(Point2f& xc);
  /**
   * @brief projects point from image to origin coordinate system
   * 
   * @param xc input image in image coordinates
   * @return Point2f output image in origin coordinate system
   */
  Point2f image2origin(Point2f& xc);
  std::vector<double> getEulerAngles(const nav_msgs::Odometry::ConstPtr& Pose);

  // This file defines the controller parameters
  camera front_cam;
  camera back_cam;

  int cnt_off;

  visual_crop_row_navigation::vs_msg VSMsg;

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
