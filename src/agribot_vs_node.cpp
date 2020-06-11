/***************************************************************************************/
/* Paper: Visual-Servoing based Navigation for Monitoring Row-Crop Fields              */
/*    Alireza Ahmadi, Lorenzo Nardi, Nived Chebrolu, Chis McCool, Cyrill Stachniss     */
/*         All authors are with the University of Bonn, Germany                        */
/* maintainer: Alireza Ahmadi                                                          */
/*          (Alireza.Ahmadi@uni-bonn.de / http://alirezaahmadi.xyz)                    */
/***************************************************************************************/
#include <ros/ros.h>
#include "agribot_vs_nodehandler.h"
#include "agribot_vs.h"

#include "std_msgs/String.h"
#include <sstream>
#include <time.h>

struct RunOnce {
  template <typename T>
  RunOnce(T &&f) { f(); }
};

int main(int argc, char** argv) {
  // initialize node
  ros::init(argc, argv, "agribot_vs");

  // node handler
  ros::NodeHandle nodeHandle;
  agribot_vs::AgribotVSNodeHandler vs_NodeH(nodeHandle);
  ros::Rate loop_rate(vs_NodeH.agribotVS.fps);

  agribot_vs::camera *I_primary,*I_secondary;
  
  if(vs_NodeH.agribotVS.camera_ID == 1){
    I_primary = &vs_NodeH.agribotVS.front_cam;
    I_secondary = &vs_NodeH.agribotVS.back_cam;
  }else {
    I_primary = &vs_NodeH.agribotVS.back_cam;
    I_secondary = &vs_NodeH.agribotVS.front_cam;
  }
  vs_NodeH.agribotVS.initialize_neigbourhood(*I_primary);
  vs_NodeH.agribotVS.initialize_neigbourhood(*I_secondary);
  int rowNum = 0;
  
  while(ros::ok()){

    if(rowNum < vs_NodeH.agribotVS.max_row_num){

      if(vs_NodeH.agribotVS.single_camera_mode){
        static RunOnce a([]() { cout << "Single Camera Mode (Front Camera will only be used..)" << endl; });
        I_secondary = I_primary;
      }
      if(!vs_NodeH.agribotVS.mask_tune){
        
        vs_NodeH.agribotVS.switching_controller(*I_primary, *I_secondary, vs_NodeH.agribotVS.min_points_switch);

        vs_NodeH.configCameras(I_primary, I_secondary);

        vs_NodeH.agribotVS.compute_feature_point(*I_primary);
        vs_NodeH.agribotVS.Controller(*I_primary,*I_secondary);
      
        if(!I_primary->image.empty()){
          vs_NodeH.agribotVS.draw_neighbourhood(*I_primary);
          vs_NodeH.agribotVS.draw_features(*I_primary, vs_NodeH.agribotVS.F_des, cv::Scalar(0, 255, 0));
          vs_NodeH.agribotVS.draw_features(*I_primary, vs_NodeH.agribotVS.F, cv::Scalar(0, 0, 255));

          // draw plant centers in image (in neighbourhood)
          for(size_t i = 0; i < I_primary->nh_points.size(); i++){
            cv::circle(I_primary->image, Point(I_primary->nh_points[i].x,I_primary->nh_points[i].y),5, Scalar(0, 204, 255), CV_FILLED, 8,0);
          }

          Mat des_comp;
          cv::resize(I_primary->image, des_comp, cv::Size(), vs_NodeH.agribotVS.Scale, vs_NodeH.agribotVS.Scale);
          imshow("Cameras", des_comp);
          waitKey(1);
        }
        
      }else{
         static RunOnce a([]() {cout << "Mask Tune Mode ..." << endl; });
      }

      if(I_primary->points.size() == 0){
        vs_NodeH.publishVelocity(0);
      }else{
        vs_NodeH.publishVelocity();
      }
      
      ros::Time curr_time = ros::Time::now();
      rosgraph_msgs::Clock curr_time_msg;
      curr_time_msg.clock = curr_time;      
      vs_NodeH.Time_pub.publish(curr_time_msg);
    }
    
    rowNum++;
    ros::spinOnce();
    loop_rate.sleep();
  }
 
  return 0;
}
