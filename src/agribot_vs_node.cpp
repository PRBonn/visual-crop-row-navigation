#include <ros/ros.h>
#include "agribot_vs_nodehandler.h"
#include "agribot_vs.h"

#include "std_msgs/String.h"
#include <sstream>
#include <time.h>



int main(int argc, char** argv) {
  // initialize node
  ros::init(argc, argv, "agribot_vs");

  // node handler
  ros::NodeHandle nodeHandle;
  ros::Rate loop_rate(10);

  agribot_vs::AgribotVSNodeHandler AgribotVSNodeHandler(nodeHandle);

  if(!AgribotVSNodeHandler.AgriBotVS.mask_tune)cout << "Mask Tune Mode ..." << endl;
  if(AgribotVSNodeHandler.AgriBotVS.single_camera_mode)cout << "Single Camera Mode ..." << endl;

  agribot_vs::camera *I_primary,*I_secondary;
  if(AgribotVSNodeHandler.AgriBotVS.cam_num == 1){
    I_primary = &AgribotVSNodeHandler.AgriBotVS.front_cam;
    I_secondary = &AgribotVSNodeHandler.AgriBotVS.back_cam;
  }else {
    I_primary = &AgribotVSNodeHandler.AgriBotVS.back_cam;
    I_secondary = &AgribotVSNodeHandler.AgriBotVS.front_cam;
  }
  AgribotVSNodeHandler.AgriBotVS.initialize_neigbourhood(*I_primary);
  AgribotVSNodeHandler.AgriBotVS.initialize_neigbourhood(*I_secondary);
  int cnt =0;
  
  while(ros::ok()){

    if(cnt > AgribotVSNodeHandler.AgriBotVS.cnt_off){

      if(AgribotVSNodeHandler.AgriBotVS.single_camera_mode){
        I_secondary = I_primary;
      }
      if(!AgribotVSNodeHandler.AgriBotVS.mask_tune){
      
        AgribotVSNodeHandler.AgriBotVS.switching_controller(*I_primary, *I_secondary, AgribotVSNodeHandler.AgriBotVS.min_points_switch);
        //cout << "Switching Controller ..." << endl;

        if(AgribotVSNodeHandler.AgriBotVS.cam_num == 1){
          I_primary = &AgribotVSNodeHandler.AgriBotVS.front_cam;
          I_secondary = &AgribotVSNodeHandler.AgriBotVS.back_cam;
        }else {
          I_primary = &AgribotVSNodeHandler.AgriBotVS.back_cam;
          I_secondary = &AgribotVSNodeHandler.AgriBotVS.front_cam;
        }

        AgribotVSNodeHandler.AgriBotVS.compute_feature_point(*I_primary);
        //cout << "compute_feature_point_front done ..." << endl;

        AgribotVSNodeHandler.AgriBotVS.Controller(*I_primary,*I_secondary);
        //cout << "Controller done ..." << endl;
      }
      if(!I_primary->image.empty()){
        AgribotVSNodeHandler.AgriBotVS.draw_neighbourhood(*I_primary);
        AgribotVSNodeHandler.AgriBotVS.draw_features(*I_primary, AgribotVSNodeHandler.AgriBotVS.F_des, cv::Scalar(0, 255, 0));
        AgribotVSNodeHandler.AgriBotVS.draw_features(*I_primary, AgribotVSNodeHandler.AgriBotVS.F, cv::Scalar(0, 0, 255));

        // draw plant centers in image (in neighbourhood)
        for(size_t i = 0; i < I_primary->nh_points.size(); i++){
          cv::circle(I_primary->image, Point(I_primary->nh_points[i].x,I_primary->nh_points[i].y),5, Scalar(0, 204, 255), CV_FILLED, 8,0);
        }
        
        //imshow("Primary Camera", I_primary->image);
        // imshow("Secondary Camera", I_secondary->image);
        //Mat Comb_cam;
        //hconcat(I_primary->image,I_secondary->image,Comb_cam);
        // Input image scaling
        Mat des_comp;
        cv::resize(I_primary->image, des_comp, cv::Size(), AgribotVSNodeHandler.AgriBotVS.Scale, AgribotVSNodeHandler.AgriBotVS.Scale);
        imshow("Cameras", des_comp);
        waitKey(1);
      }else{
        cout << "Is  Image empty? 1: " << I_primary->image.empty() <<  " , 2: "<<I_secondary->image.empty() << endl;
      }
    if(I_primary->points.size() == 0){
        cout << "X: " << AgribotVSNodeHandler.AgriBotVS.VelocityMsg.linear.x << 
        " , Z: "<< AgribotVSNodeHandler.AgriBotVS.VelocityMsg.angular.z << endl;
        AgribotVSNodeHandler.PublishVelocity(0);
      }else{
        AgribotVSNodeHandler.PublishVelocity();
      }
      
      ros::Time curr_time = ros::Time::now();
      rosgraph_msgs::Clock curr_time_msg;
      curr_time_msg.clock = curr_time;      
      AgribotVSNodeHandler.Time_pub.publish(curr_time_msg);
    }
    
    if(cnt < 1000)cnt++;
    //cout << "X: " << AgribotVSNodeHandler.AgriBotVS.VelocityMsg.linear.x << 
    //    " , Z: "<< AgribotVSNodeHandler.AgriBotVS.VelocityMsg.angular.z << endl;
    ros::spinOnce();
    loop_rate.sleep();
  }
 
  return 0;
}
