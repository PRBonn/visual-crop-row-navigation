#include "agribot_vs_nodehandler.h"
#include "agribot_vs.h"
#include <time.h>

// #include <agribot_visualservoing/AgribotVSConfig.h>

namespace agribot_vs {

AgribotVSNodeHandler::AgribotVSNodeHandler(ros::NodeHandle& nodeHandle): nodeHandle_(nodeHandle){
  ROS_ERROR("Visual Servoing core is running...");
  if (!AgriBotVS.ReadConfig_run(nodeHandle_)) {
     ROS_ERROR("Could not read parameters.");
     ros::requestShutdown();
  }

  // subsribe topic
  // image_front_sub = nodeHandle_.subscribe("/agribot/front_camera/image_raw", 2, &AgribotVSNodeHandler::image_front_calllback,this);
  // image_back_sub = nodeHandle_.subscribe("/agribot/back_camera/image_raw", 2, &AgribotVSNodeHandler::image_back_calllback,this);
  image_front_sub = nodeHandle_.subscribe("/front/rgb/image_raw", 2, &AgribotVSNodeHandler::image_front_calllback,this);
  image_back_sub = nodeHandle_.subscribe("/back/rgb/image_raw", 2, &AgribotVSNodeHandler::image_back_calllback,this);
  Mocap_sub = nodeHandle_.subscribe("/amcl_pose", 1, &AgribotVSNodeHandler::Amcl_PoseCallback,this);
  Odom_sub = nodeHandle_.subscribe("/odometry/raw", 10, &AgribotVSNodeHandler::OdomCallback,this);
  IMU_sub = nodeHandle_.subscribe("/imu/data", 1000, &AgribotVSNodeHandler::IMUCallback,this);
  
  Time_pub = nodeHandle_.advertise<rosgraph_msgs::Clock>("/clock", 10);
  VSVelocityPub = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  Log_pub = nodeHandle_.advertise<agribot_visualservoing::VisualServoingMsg>("/vs_msg", 10);
  _cnt = 0;
  // dynamic_reconfigure::Server<agribot_visualservoing::AgribotVSConfig> srv;
  // dynamic_reconfigure::Server<agribot_visualservoing::AgribotVSConfig>::CallbackType f;
  // f = boost::bind(&dynamicReconfig_callback, _1, _2);
  // srv.setCallback(f);

  AgriBotVS.VelocityMsg.linear.x =0.0;
  AgriBotVS.VelocityMsg.angular.z =0.0;

  state = 0;
  in_state = 0;
}

AgribotVSNodeHandler::~AgribotVSNodeHandler() {
}

void AgribotVSNodeHandler::CropRow_Tracking(camera& src){
  if(AgriBotVS.ControllerType==0){    // Old version of controller PID based
    // if(state == 0){
    //   Mat image;
    //   AgriBotVS.Scale = 0.6;
    //   cv::resize(src, image, cv::Size(), AgriBotVS.Scale, AgriBotVS.Scale);
    //   AgriBotVS.CenterLine[0][0] = image.cols / 2;
    //   AgriBotVS.CenterLine[0][1] = 0;
    //   AgriBotVS.CenterLine[0][2] = image.cols / 2;
    //   AgriBotVS.CenterLine[0][3] = image.rows;

    //   AgriBotVS.center_max = image.cols / 2 + AgriBotVS.center_max_off;
    //   AgriBotVS.center_min = image.cols / 2 - AgriBotVS.center_min_off;

    //   vector<vector<Point>> contours = AgriBotVS.CropRowFeatures(image);
    //   vector<Point2f> contour_centers = AgriBotVS.getContureCenters(image, contours);
    //   vector<Vec4i> lines_Vert;
    //   if(AgriBotVS.LineFitting_method == 0){    // HoughLinesP line fitting
    //     lines_Vert = AgriBotVS.HouphLineOnImage(AgriBotVS.img_contour, contour_centers);
    //   }else if(AgriBotVS.LineFitting_method == 1){  // fitLine - Regresion based line fitting
    //     lines_Vert = AgriBotVS.FitLineOnContures(AgriBotVS.img_contour, contour_centers);
    //   }
    //   if(contours.size() == 0 ){
    //     state++; // there is no line !!
    //   }
    //   AgriBotVS.ComputeVelocities(image, AgriBotVS.AngleDifference(image, lines_Vert), lines_Vert);
      
    //   PublishVelocity();
    // }else if (state == 1){
    //   cout << "Row is ended!!" << endl;
    //   StopForSec(2);
    //   cout << "Going toward secont check-point 0..." << endl;
    //   if (AgriBotVS.SetNewGoal(AgriBotVS.x_poses[0], AgriBotVS.y_poses[0], AgriBotVS.z_poses[0])) {
    //     ROS_INFO("First check-point reached ...");
    //     StopForSec(1);
    //     state++;
    //   }
    // }else if (state == 2){
    //   cout << "Turn in Place -90deg ..." << endl;
    //   TurnInPlace(-1.54);
    //   ROS_INFO("%d",state);
    // }else if (state == 3){
    //   StopForSec(2);
    //   cout << "Going toward secont check-point 1..." << endl;
    //   if (AgriBotVS.SetNewGoal(AgriBotVS.x_poses[1], AgriBotVS.y_poses[1], AgriBotVS.z_poses[1])) {
    //     ROS_INFO("First check point reached...");
    //     StopForSec(1);
    //     state++;
    //   }
    // }else if (state == 4){
    //   cout << "Turn in Place -90deg ..." << endl;
    //   TurnInPlace(-3.14);
    // }else if (state == 5){
    //   StopForSec(2);
    //   cout << "Going toward secont check-point 2..." << endl;
    //   if (AgriBotVS.SetNewGoal(AgriBotVS.x_poses[2], AgriBotVS.y_poses[2], AgriBotVS.z_poses[2])) {
    //     ROS_INFO("First check point reached...");
    //     StopForSec(1);
    //     state++;
    //   }
    // }else if (state == 6){
    //   // supposed to be in start of another croprow
    //   state = 0;
    // }
  }else{                              // Visual Servoing implementation
    
    // finding contour from image baed on crops in rows
    src.contours = AgriBotVS.CropRowFeatures(src.image);
    //cout << "CropRowFeatures done ..." << endl;
    if(!AgriBotVS.mask_tune || src.contours.size() != 0){

      src.points = AgriBotVS.getContureCenters(src.image, src.contours);
      
      src.nh_points = AgriBotVS.filterContures(src.image, src.contours);

      AgriBotVS.is_in_neigbourhood(src);
      //cout << "getContureCenters done ..." << endl;

      src.lines =  AgriBotVS.FitLineOnContures(src.image, src.nh_points);
      //cout << "FitLineOnContures done ...  " << endl;
    }else{
      cout << "Numner of contures: " << src.contours.size() << endl;
      PublishVelocity(0);
    }
  }
}

void AgribotVSNodeHandler::image_front_calllback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    AgriBotVS.front_cam.image = cv_bridge::toCvShare(msg, "bgr8")->image;
    CropRow_Tracking(AgriBotVS.front_cam);
    
    string str;
    stringstream stream,stream1,stream2;
    stream << AgriBotVS.front_cam.points.size(); 
    stream1 << AgriBotVS.front_cam.nh_points.size(); 
    stream2 << AgriBotVS.cam_num;
    str = "Number of Points: " + stream.str() + " nh_points: " + stream1.str() + " Cam ID: " + stream2.str();
    cv::putText(AgriBotVS.front_cam.image, str, cv::Point(40, 20),  // Coordinates
                cv::FONT_HERSHEY_COMPLEX_SMALL,       // Font
                0.75,                                 // Scale. 2.0 = 2x bigger
                cv::Scalar(0, 0, 255),                // BGR Color
                1);                                   // Line Thickness (Optional)

  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void AgribotVSNodeHandler::image_back_calllback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    AgriBotVS.back_cam.image = cv_bridge::toCvShare(msg, "bgr8")->image;
    CropRow_Tracking(AgriBotVS.back_cam);

    string str;
    stringstream stream,stream1,stream2;
    stream << AgriBotVS.back_cam.points.size(); 
    stream1 << AgriBotVS.back_cam.nh_points.size(); 
    stream2 << AgriBotVS.cam_num;
    str = "Number of Points: " + stream.str() + " nh_points: " + stream1.str() + " Cam ID: " + stream2.str();
    cv::putText(AgriBotVS.back_cam.image, str, cv::Point(40, 20),  // Coordinates
                cv::FONT_HERSHEY_COMPLEX_SMALL,       // Font
                0.75,                                 // Scale. 2.0 = 2x bigger
                cv::Scalar(0, 0, 255),                // BGR Color
                1);                                   // Line Thickness (Optional)

  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void AgribotVSNodeHandler::TurnInPlace(float angle){
  double speed; 
  if(in_state == 0){
    ROS_INFO("TurnInPlace %d",in_state);
    AgriBotVS.VelocityMsg.linear.y = 1.0;
    AgriBotVS.VelocityMsg.linear.x = 0.0;
    AgriBotVS.VelocityMsg.angular.z = 0.0;
    PublishVelocity();
    ros::Duration(6).sleep();
    in_state++;
  }else if(in_state == 1){
    double diff = (angle - mocap_yaw);
    ROS_INFO("TurnInPlace %d",in_state);
    ROS_INFO("diff: %f, angle: %f, mocap_yaw: %f", diff, angle, mocap_yaw);

    speed = diff/4;
    if(speed < 0.0  && speed > - 0.4){speed = - 0.4;}
    if(speed > 0.0 && speed < 0.4){speed = 0.4;}

    AgriBotVS.VelocityMsg.angular.z = speed;
    PublishVelocity();

    if(abs(diff) < 0.03){
      AgriBotVS.VelocityMsg.linear.y = 0.0;
      AgriBotVS.VelocityMsg.linear.x = 0.0;
      AgriBotVS.VelocityMsg.angular.z = 0.0;
      PublishVelocity();
      ros::Duration(0.5).sleep();
      in_state++;
    }
  }else if(in_state == 2){
    ROS_INFO("TurnInPlace %d",in_state);
    AgriBotVS.VelocityMsg.angular.z = 0;
    PublishVelocity();
    ros::Duration(6).sleep();
    in_state = 0;
    state++;
  }
}

void AgribotVSNodeHandler::IMUCallback(const sensor_msgs::Imu::ConstPtr& msg){
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->orientation, quat);
  //ROS_INFO("Imu Seq: [%d]", msg->header.seq);
  //ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
  tf::Matrix3x3 m(quat);
  m.getRPY(imu_roll, imu_pitch, imu_yaw);
  //ROS_INFO("Yaw: %f",imu_yaw);
}

void AgribotVSNodeHandler::Amcl_PoseCallback(const geometry_msgs::PoseStamped& msg) {
  tf::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y,
                   msg.pose.orientation.z, msg.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(mocap_roll, mocap_pitch, mocap_yaw);
  // cout << "Yaw is : " << mocap_yaw << endl;
  AgriBotVS.RotationVec[0] = 0;    // with respect to camera orientation
  AgriBotVS.RotationVec[1] = 0;    // with respect to camera orientation
  AgriBotVS.RotationVec[2] = mocap_yaw;  // with respect to robot orientation
  AgriBotVS.TransVec[0] = msg.pose.position.x;
  AgriBotVS.TransVec[1] = msg.pose.position.y;
  AgriBotVS.TransVec[2] = msg.pose.position.z;
}

void AgribotVSNodeHandler::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  AgriBotVS.RobotPose[0] = msg->pose.pose.position.x;
  AgriBotVS.RobotPose[1] = msg->pose.pose.position.y;
  AgriBotVS.RobotPose[2] = msg->pose.pose.position.z;

  std::vector<double> Orineration = AgriBotVS.getEulerAngles(msg);

  AgriBotVS.RobotPose[3] = Orineration[0];  // x oreintation
  AgriBotVS.RobotPose[4] = Orineration[1];  // y oreintation
  AgriBotVS.RobotPose[5] = Orineration[2];  // z oreintation

  AgriBotVS.RobotLinearVelocities[0] = msg->twist.twist.linear.x;
  AgriBotVS.RobotLinearVelocities[1] = msg->twist.twist.linear.y;
  AgriBotVS.RobotLinearVelocities[2] = msg->twist.twist.linear.z;

  AgriBotVS.RobotAngularVelocities[0] = msg->twist.twist.angular.x;
  AgriBotVS.RobotAngularVelocities[1] = msg->twist.twist.angular.y;
  AgriBotVS.RobotAngularVelocities[2] = msg->twist.twist.angular.z;
  // ROS_INFO("Seq: [%d]", msg->header.seq);
  // ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]",
  // msg->pose.pose.position.x,msg->pose.pose.position.y,
  // msg->pose.pose.position.z); ROS_INFO("Orientation-> x: [%f], y: [%f], z:
  // [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
  // msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  // ROS_INFO("Vel-> Linear: [%f], Angular: [%f]",
  // msg->twist.twist.linear.x,msg->twist.twist.angular.z);
}

void AgribotVSNodeHandler::StopForSec(float delay) {
  AgriBotVS.VelocityMsg.angular.z = 0.0;
  AgriBotVS.VelocityMsg.linear.x = 0.0;
  if(AgriBotVS.publish_cmd_vel)VSVelocityPub.publish(AgriBotVS.VelocityMsg);
  ros::Duration(delay).sleep();  // sleep for half a second
}

void AgribotVSNodeHandler::PublishVelocity(int _in) {
  if(!AgriBotVS.publish_linear_vel)AgriBotVS.VelocityMsg.linear.x = 0.0;
  if(_in == 0){
    AgriBotVS.VelocityMsg.linear.x = 0.0;
    AgriBotVS.VelocityMsg.angular.z = 0.0;
    if(AgriBotVS.publish_cmd_vel)
    VSVelocityPub.publish(AgriBotVS.VelocityMsg);
  }else{
    if(AgriBotVS.publish_cmd_vel){
      VSVelocityPub.publish(AgriBotVS.VelocityMsg);
    }
  }
  Log_pub.publish(AgriBotVS.VSMsg);
}

// void AgribotVSNodeHandler::dynamicReconfig_callback(agribot_visualservoing::AgribotVSConfig &config, uint32_t level){
//   ROS_INFO("Reconfigure request : %f %f %i %i %i %s %i %s %f %i",
//            config.groups.angles.min_ang,
//            config.groups.angles.max_ang,
//            (int)config.intensity,
//            config.cluster,
//            config.skip,
//            config.port.c_str(),
//            (int)config.calibrate_time,
//            config.frame_id.c_str(),
//            config.time_offset,
//            (int)config.allow_unsafe_settings);

//   // do nothing for now
// }

}   // namespace agribot_vs
