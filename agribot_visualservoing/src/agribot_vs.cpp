/**************************************************************************/
/* Paper: Visual-Servoing based Navigation for Monitoring Row-Crop Fields */
/*    Alireza Ahmadi, Lorenzo Nardi, Nived Chebrolu, Cyrill Stachniss     */
/*         All authors are with the University of Bonn, Germany           */
/* maintainer: Alireza Ahmadi                                             */
/*          (Alireza.Ahmadi@uni-bonn.de / http://alirezaahmadi.xyz)       */
/**************************************************************************/

#include "agribot_vs.h"

using namespace cv;
using namespace std;
using namespace Eigen;

using std::cerr;
using cv::Mat;

namespace agribot_vs{

  AgribotVS::AgribotVS(){

    CenterLine.resize(1, 0);
    CenterLine[0][0] = 1280 / 2;
    CenterLine[0][1] = 0;
    CenterLine[0][2] = 1280 / 2;
    CenterLine[0][3] = 720;

    RobotPose.resize(6, 0);
    RobotLinearVelocities.resize(3, 0);
    RobotAngularVelocities.resize(3, 0);
    RotationVec.resize(3, 0);
    TransVec.resize(3, 0);

    minp_cnt=0;

    id = 0;

    navigation_dir = 1;


    F_des << 0,0,0;
    F << 0,0,0;
  }
  AgribotVS::~AgribotVS(){
  };

  bool AgribotVS::readRUNParmas(ros::NodeHandle& nodeHandle_) {

    nodeHandle_.param("/agribot_vs/min_frame", min_frame, 15);
    
    nodeHandle_.param("/agribot_vs/coef", coef, 60.0);

    nodeHandle_.param("/agribot_vs/ex_Xc", ex_Xc, 640);
    nodeHandle_.param("/agribot_vs/ex_Yc", ex_Yc, 360);
    nodeHandle_.param("/agribot_vs/nh_offset", nh_offset, 200);
    nodeHandle_.param("/agribot_vs/nh_L", nh_L, 150);
    nodeHandle_.param("/agribot_vs/nh_H", nh_H, 250);

    nodeHandle_.param("/agribot_vs/minContourSize", minContourSize, 8.0);
    
    nodeHandle_.param("/agribot_vs/cnt_off", cnt_off, 80);

    nodeHandle_.param("/agribot_vs/mode", mode, 1);

    nodeHandle_.param("/agribot_vs/mask_tune", mask_tune, false);
    nodeHandle_.param("/agribot_vs/single_camera_mode", single_camera_mode, false);

    nodeHandle_.param("/agribot_vs/z_min", z_min, 0.15);

    nodeHandle_.param("/agribot_vs/w_min", w_min, 0.015);
    nodeHandle_.param("/agribot_vs/w_max", w_max, 0.15);
    nodeHandle_.param("/agribot_vs/del_t", del_t, 0.1);
    nodeHandle_.param("/agribot_vs/vf_des", vf_des, 0.05);
    nodeHandle_.param("/agribot_vs/vb_des", vb_des, 0.05);

    nodeHandle_.param("/agribot_vs/ty", ty, 0.1);
    nodeHandle_.param("/agribot_vs/tz", tz, 1.034);

    nodeHandle_.param("/agribot_vs/rho_f", rho_f, -70.0);
    nodeHandle_.param("/agribot_vs/rho_b", rho_b, -40.0);
    

    nodeHandle_.param("/agribot_vs/cam_num", cam_num, 1);

    nodeHandle_.param("/agribot_vs/lambda_x_1", lambda_x_1, 1.0);
    nodeHandle_.param("/agribot_vs/lambda_w_1", lambda_w_1, 1.0);

    nodeHandle_.param("/agribot_vs/lambda_x_2", lambda_x_2, 1.0);
    nodeHandle_.param("/agribot_vs/lambda_w_2", lambda_w_2, 1.0);

    nodeHandle_.param("/agribot_vs/lambda_x_3", lambda_x_3, 1.0);
    nodeHandle_.param("/agribot_vs/lambda_w_3", lambda_w_3, 1.0);

    nodeHandle_.param("/agribot_vs/lambda_x_4", lambda_x_4, 1.0);
    nodeHandle_.param("/agribot_vs/lambda_w_4", lambda_w_4, 1.0);

    nodeHandle_.param("/agribot_vs/height", height, 720);
    nodeHandle_.param("/agribot_vs/width", width, 1280);

    nodeHandle_.param("/agribot_vs/drive_forward", drive_forward, true);
    nodeHandle_.param("/agribot_vs/turning_mode", turning_mode, false);
    nodeHandle_.param("/agribot_vs/steering_dir", steering_dir, 1);
    nodeHandle_.param("/agribot_vs/driving_dir", driving_dir, 1);

    nodeHandle_.param("/agribot_vs/min_points_switch", min_points_switch, 15);
    
    nodeHandle_.param("/agribot_vs/publish_cmd_vel", publish_cmd_vel, true);
    nodeHandle_.param("/agribot_vs/publish_linear_vel", publish_linear_vel, true);

    nodeHandle_.param("/agribot_vs/max_Hue", max_Hue, 80);
    nodeHandle_.param("/agribot_vs/min_Hue", min_Hue, 10);

    nodeHandle_.param("/agribot_vs/max_Saturation", max_Saturation, 255);
    nodeHandle_.param("/agribot_vs/min_Saturation", min_Saturation, 100);

    nodeHandle_.param("/agribot_vs/max_Value", max_Value, 255);
    nodeHandle_.param("/agribot_vs/min_Value", min_Value, 100);

    nodeHandle_.param("/agribot_vs/Scale", Scale, 0.5);

    cout << "Run parameters loading ..." << endl;

    rho_b *=DEG2RAD;
    rho_f *=DEG2RAD;

    return true;
  }
  vector<vector<Point>> AgribotVS::CropRowFeatures(Mat& img) {
    // // convert to HSV color space
    cv::Mat hsvImage;
    cv::cvtColor(img, hsvImage, CV_BGR2HSV);
    // split the channels
    std::vector<cv::Mat> hsvChannels;
    cv::split(hsvImage, hsvChannels);

    // is the color within the lower hue range?
    cv::Mat hueMask;
    cv::Mat hueImg = hsvChannels[0];
    cv::inRange(hueImg, min_Hue, max_Hue, hueMask);
    cv::Mat saturationMask;
    cv::Mat saturationImg = hsvChannels[1];
    cv::inRange(saturationImg, min_Saturation, max_Saturation, saturationMask);
    cv::Mat valueMask;
    cv::Mat valueImg = hsvChannels[2];
    cv::inRange(valueImg, min_Value, max_Value, valueMask);

    hueMask = hueMask & saturationMask & valueMask;
    
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    RNG rng(12345);

    // finds cluster/contour in the image
    findContours(hueMask, contours, hierarchy, CV_RETR_TREE,
                CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    // Draw contours
    img_contour = Mat::zeros(hueMask.size(), CV_8UC3);
    Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    for(size_t i = 0; i < contours.size(); i++){
      drawContours(img_contour, contours, i, color, 1, 8, hierarchy, 0, Point());
    }

    if(mask_tune){
      Mat Comb_HSV;
      hconcat(hueMask,saturationMask,Comb_HSV);
      hconcat(Comb_HSV,valueMask ,Comb_HSV);
      cv::resize(Comb_HSV, Comb_HSV, cv::Size(), Scale, Scale);
      cv::imshow("HSV image", Comb_HSV);

      cv::resize(img_contour, img_contour, cv::Size(), Scale, Scale);
      imshow("Contours", img_contour);
      waitKey(1);
    }
    return contours;
  }
  vector<Point2f> AgribotVS::getContureCenters(Mat& img, vector<vector<Point>>& contours){
    /// Approximate contours to polygons + get bounding rects and circles
    vector<vector<Point>> contours_poly(contours.size());
    vector<Point2f> center(contours.size());
    vector<float> radius(contours.size());

    // find enclosing Polygon whcih fits arround the contures 
    for (size_t i = 0; i < contours.size(); i++) {
      // for(size_t j = 0; j < contours[i].size(); j++)
      // {
      //   center.push_back(Point2f(contours[i][j].x,contours[i][j].y));
      // }
      
      approxPolyDP(Mat(contours[i]), contours_poly[i], 2, true);
      minEnclosingCircle((Mat)contours_poly[i], center[i], radius[i]);
      cv::circle(img, Point(center[i].x, center[i].y),3, Scalar(51, 204, 51),CV_FILLED, 8,0);
    }
    return center;
  }
  vector<Point2f> AgribotVS::filterContures(Mat& img, vector<vector<Point>>& contours){
    /// Approximate contours to polygons + get bounding rects and circles
    vector<vector<Point>> contours_poly(contours.size());
    vector<Point2f> center(contours.size());
    vector<float> radius(contours.size());
    vector<Point2f> Filtered_Centers;
    RNG rng(12345);
    // find enclosing Polygon whcih fits arround the contures 
    for (size_t i = 0; i < contours.size(); i++) {
      approxPolyDP(Mat(contours[i]), contours_poly[i], 1, true);
      minEnclosingCircle((Mat)contours_poly[i], center[i], radius[i]);
      if (center[i].x >= (img.cols/2) - center_min_off && 
          center[i].x <= (img.cols/2) + center_max_off && 
          radius[i] >= this->minContourSize) {
        Filtered_Centers.push_back(center[i]);
        //Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
        //cv::circle(img, Point(center[i].x, center[i].y),15, color, 3,0); //CV_FILLED  Scalar(100,50,100)
      }

    }
    return Filtered_Centers;
  }
  vector<Vec4i> AgribotVS::FitLineOnContures(Mat& img, vector<Point2f>& ContourCenters){

    
    Point2f P1;
    Point2f P2;
    vector<Vec4i> AvgLine;
    AvgLine.resize(1);
   
    if(ContourCenters.size() > 0 ){
      Vec4f linefit;
      cv::fitLine(ContourCenters,linefit,CV_DIST_L2,0,0.01,0.01);
      // float vx = linefit[0];
      // float vy = linefit[1];
      // x = linefit[2];
      // y = linefit[3];
      // int lefty = round((-x * vy / vx) + y);
      // int righty = round(((img.cols - x) * vy / vx) + y);
      // line[0] = img.cols - 1;
      // line[1] = righty;
      // line[2] = 0;
      // line[3] = lefty;

      P1.x = linefit[0] + linefit[2];
      P1.y = linefit[1] + linefit[3];
      P2.x = linefit[2];
      P2.y = linefit[3];

      Vector3f top_left(0,0,1);
      Vector3f top_right(width,0,1);
      Vector3f bottom_left(0,height,1);
      Vector3f bottom_right(width,height,1);

      // compute image border lines
      Vector3f l_ib = bottom_left.cross(bottom_right); 
      Vector3f l_it = top_left.cross(top_right);
      Vector3f l_il = top_left.cross(bottom_left);
      Vector3f l_ir = top_right.cross(bottom_right);

      // compute line PQ
      Vector3f P_h(P1.x,P1.y,1);
      Vector3f Q_h(P2.x,P2.y,1);
      Vector3f l = P_h.cross(Q_h);

      // compute intersections with all four lines
      Vector2f R_t = hom2euc(l.cross(l_it));
      Vector2f R_l = hom2euc(l.cross(l_il));
      Vector2f R_b = hom2euc(l.cross(l_ib)); 
      Vector2f R_r = hom2euc(l.cross(l_ir));

      // compute points within the image 
      MatrixXf R(4,2);
      R << R_t(0), R_t(1),
          R_l(0), R_l(1),
          R_b(0), R_b(1),
          R_r(0), R_r(1);

      MatrixXf R_out(2,2);
      R_out = is_in_image_point(R);

      // if(Vectlines.size() >= (uint)FilterQuieSize){
      // Vectlines.erase(Vectlines.begin());
      // }
      // Vectlines.push_back(line);

      // for (uint i = 0; i < Vectlines.size(); i++)
      // {
      //   AvgLine[0][0] += Vectlines[i][0];
      //   AvgLine[0][1] += Vectlines[i][1];
      //   AvgLine[0][2] += Vectlines[i][2];
      //   AvgLine[0][3] += Vectlines[i][3];
      // }
      // AvgLine[0][0] /= FilterQuieSize;
      // AvgLine[0][1] /= FilterQuieSize;
      // AvgLine[0][2] /= FilterQuieSize;
      // AvgLine[0][3] /= FilterQuieSize;

      // Vec4i l = AvgLine[0];
      //cout << R_out(0,0) << " "<< R_out(0,1) << " "<<  R_out(1,0) << " "<< R_out(1,1) << " " << endl;
      cv::line(img, Point(R_out(0,0), R_out(0,1)),Point(R_out(1,0), R_out(1,1)), Scalar(0, 0, 255), 1, CV_AA);
      AvgLine[0][0] =R_out(0,0);
      AvgLine[0][1] =R_out(0,1);
      AvgLine[0][2] =R_out(1,0);
      AvgLine[0][3] =R_out(1,1);
    }
    return AvgLine;
  }
  //*************************************************************************************************
  void AgribotVS::Controller(camera& I_primary, camera& I_secondary){
    // id = 1 => Row controller
    // id = 2 => Column controller
    // input feature

    float X = F(0); 
    float Y = F(1); 
    float Theta = F(2);

    float v = 0;
    if( driving_dir ==1){
      v = driving_dir*vf_des;
    }else{
      v = driving_dir*vb_des;
    }
    // compute interaction matrix

    MatrixXf lambda(2,1);
    if(driving_dir == 1){
      if(steering_dir == 1){ // mode 1
        lambda << lambda_x_1,lambda_w_1;
        rho = rho_f;
        mode = 1;
        //ty = 0.0;
      }else{                 // mode 2
        lambda << lambda_x_2,lambda_w_2;
        rho = rho_b;
        mode = 2;
        //ty = 1.08;
      }
    }else{
      if(steering_dir == -1){ // mode 3
        lambda << lambda_x_3,lambda_w_3;
        rho = rho_b;
        mode = 3;
        //ty = 1.08;
      }else{                  // mode 4
        lambda << lambda_x_4,lambda_w_4;
        rho = rho_f;
        mode = 4;
        //ty = 0.0;
      }
    }

    MatrixXf Ls(3,6);
      Ls << -(sin(rho)+Y*cos(rho))/tz, 0, X*(sin(rho)+Y*cos(rho))/tz, X*Y, -1-pow(X,2),  Y,
            0,   -(sin(rho)+Y*cos(rho))/tz, Y*(sin(rho)+Y*cos(rho))/tz, 1+pow(Y,2), -X*Y, -X,
            cos(rho)*pow(cos(Theta),2)/tz, cos(rho)*cos(Theta)*sin(Theta)/tz, -(cos(rho)*cos(Theta)*(Y*sin(Theta) + X*cos(Theta)))/tz, -(Y*sin(Theta) + X*cos(Theta))*cos(Theta), -(Y*sin(Theta) + X*cos(Theta))*sin(Theta), -1;

    // compute tranformation between robot to camera frame
    MatrixXf c(2,6); 
    if( cam_num == 1){
      c << 0, -sin(rho), cos(rho), 0, 0, 0,
          -ty, 0, 0, 0, -cos(rho ), -sin(rho);
    }else{
      c << 0, sin(rho), -cos(rho), 0, 0, 0,
        -ty, 0, 0, 0, cos(rho), sin(rho);
    }

    MatrixXf cTR(6,2);
      cTR = c.transpose();

    MatrixXf Tv(6,1);
      Tv = cTR.col(0);
    
    MatrixXf Tw(6,1);
      Tw = cTR.col(1);

    // compute Jacobian
    MatrixXf Jv(2,6);
      Jv << Ls.row(id),
            Ls.row(2);
      Jv = Jv*Tv;

    MatrixXf Jw(2,6);
      Jw << Ls.row(id),
            Ls.row(2);
      Jw = Jw*Tw;

    // // compute control law
    Vector2f err((F[id] - F_des[id]), wrapToPi(F[2] - F_des[2]));

    // set weights
    MatrixXf tmp_lambda(2,1);
      tmp_lambda << lambda(0)*err(0),
                    lambda(1)*err(1);

    // compute control
    MatrixXf Jw_pinv(6,2);
    Jw_pinv = Jw.completeOrthogonalDecomposition().pseudoInverse();

    MatrixXf w = -Jw_pinv*(tmp_lambda + Jv*v);

    w(0,0) = copysign(min(abs(w(0,0)),(float)w_max), w(0,0));

    if(abs(v) < 0.05 ||  (I_primary.nh_points.size() < 5  && I_secondary.nh_points.size() <5)){
      VelocityMsg.angular.z = 0;
    }else{
      if(w(0,0) > 0.3) w(0,0) = 0.3;
      VelocityMsg.angular.z = steering_dir * w(0,0);
    }

    if(abs(VelocityMsg.angular.z) < z_min || mode == 4 || mode==2)VelocityMsg.angular.z=0.0;
    VelocityMsg.linear.x  = v;

    VSMsg.err_x=abs(err(0));
    VSMsg.err_theta=abs(err(1));
  }
  void AgribotVS::compute_feature_point(camera& I){

    if(drive_forward){
      P.x = I.lines[0][0];
      P.y = I.lines[0][1];
      Q.x = I.lines[0][2];
      Q.y = I.lines[0][3];
    }else{
      P.x = I.lines[0][2];
      P.y = I.lines[0][3];
      Q.x = I.lines[0][0];
      Q.y = I.lines[0][1];
    }

    I.nh_ex = Compute_nh_ex(I);
    I.nh.Xc = (I.nh_ex[0].x + I.nh_ex[1].x)/2;
    I.nh.Yc = (I.nh_ex[0].y + I.nh_ex[1].y)/2;

    // computes intersection side
    cv::circle(I.image, Point(P.x, P.y),8, Scalar(0,0,255),CV_FILLED, 12,0);
    cv::circle(I.image, Point(Q.x, Q.y),8, Scalar(0,255,255),CV_FILLED, 12,0);

    // compute Theta
    float Theta = compute_Theta(P,Q);
    
    // compute F
    Point2f _F = camera2image(P);
    F << _F.x,
         _F.y,
         Theta;
    F_des <<  0,
              height/2,
              0;
    id = 0;
  }
  void AgribotVS::switching_controller(camera& I_primary, camera& I_secondary, unsigned int min_points){

    is_in_neigbourhood(I_primary); 

    double avg_nh_points_y = 0.0;
    for (unsigned int i = 0; i < I_primary.nh_points.size(); ++i){
      avg_nh_points_y += I_primary.nh_points[i].y;
    }
    avg_nh_points_y /= I_primary.nh_points.size();

    double avg_points_y =0.0;
    for (unsigned int i = 0; i < I_primary.points.size(); ++i){
      avg_points_y += I_primary.points[i].y;
    }
    avg_points_y /= I_primary.points.size();

    if(I_primary.nh_points.size() < min_points){
      minp_cnt++;
    }else{
      minp_cnt = 0;
    }

    if((I_primary.nh_points.size() == 0 && minp_cnt >=min_frame) ||  
      (avg_nh_points_y < (double)coef && avg_points_y < (double)coef*2) || 
      (avg_nh_points_y > (double)height-coef && avg_points_y > (double)height-coef*2)){
      minp_cnt=0;
      cout << "I_primary doesn't see anything !!!! id: " << cam_num << endl;
      if(I_secondary.points.size() < min_points){
        cout << "turning_mode: " << turning_mode << endl;
          // No points visible in both cameras
          if(turning_mode){
            // 2->3 and 4->1
            mode++;
            if(mode == 5)mode=1;
            cout << "TURNING..." << endl;
            // switch steering direction
            driving_dir = -driving_dir;
            // switch behavior cameras
            drive_forward = true;
            turning_mode  = false;
            cout << "turning mode OFF" << endl;
            // shift the neigbourhood
            //steering_dir = -steering_dir;
            shift_neighbourhood(I_primary, steering_dir);
            is_in_neigbourhood(I_primary);  

          }
      }else{
          mode++;
          // 1->2 and 3->4
          cout << "SWITCHING CAMERAS" << endl;
          switch_cameras(cam_num);
          initialize_neigbourhood(I_secondary);
          initialize_neigbourhood(I_primary);
          is_in_neigbourhood(I_primary);   
          turning_mode = true;
          drive_forward = false;
          steering_dir = -steering_dir;
          cout << "turning mode ON" << endl;
      }
    }else{
        cout  << 
        "mode: " << mode << 
        ", cam: " <<  cam_num << 
        ", df: " << drive_forward << 
        ", sd: " << steering_dir << 
        ", dd: " << driving_dir << 
        ", nh_p: " << I_primary.nh_points.size() <<
         endl;
    }
  }
  //*************************************************************************************************
  int AgribotVS::compute_intersection(Point2f& P, Point2f& Q){
    Vector3f top_left(0,0,1);
    Vector3f top_right(width,0,1);
    Vector3f bottom_left(0,height,1);
    Vector3f bottom_right(width,height,1);

    // compute image border lines
    Vector3f l_ib = bottom_left.cross(bottom_right); 
    Vector3f l_it = top_left.cross(top_right);
    Vector3f l_il = top_left.cross(bottom_left);
    Vector3f l_ir = top_right.cross(bottom_right);

    // compute line PQ
    Vector3f P_h(P.x,P.y,1);
    Vector3f Q_h(Q.x,Q.y,1);
    Vector3f l =  P_h.cross(Q_h);

    // compute intersections with all four lines
    Vector2f R_t = hom2euc(l.cross(l_it));
    Vector2f R_l = hom2euc(l.cross(l_il));
    Vector2f R_b = hom2euc(l.cross(l_ib)); 
    Vector2f R_r = hom2euc(l.cross(l_ir));

    // compute points within the image 
    MatrixXf R(4,2);
    R << R_t(0), R_t(1),
         R_l(0), R_l(1),
         R_b(0), R_b(1),
         R_r(0), R_r(1);
    Vector4i in = is_in_image(R);
    
    int ind_min = 0;
    int ind_max = 0;
    int tmp_min_y = 10000;
    int tmp_max_y = 0;
    for(int i = 0; i < 4; i++){
      if(in(i) == 1){
        if(R(i,1) < tmp_min_y){
          tmp_min_y = R(i,1);
          ind_min = i;
        }
        if(R(i,1) > tmp_max_y){
          tmp_max_y = R(i,1);
          ind_max = i;
        }
      }
    }
    
    Q.x = R(ind_min,0);
    Q.y = R(ind_min,1);
    P.x = R(ind_max,0);
    P.y = R(ind_max,1);

    return ind_max;
  }
  void AgribotVS::compute_intersection_old(Point2f& P, Point2f& Q){
    // computes side given the points P, Q - side = 1 (Top) - side = 2 (Left) - side = 3 (Bottom) - side = 4 (Right)

    Vector3f origin_h(0,0,1);
    Vector3f W_h(width,0,1);
    Vector3f H_h(0,height,1);
    Vector3f WH_h(width,height,1);

    // compute image border lines
    Vector3f l_ib = origin_h.cross(W_h); 
    Vector3f l_it = H_h.cross(WH_h);
    Vector3f l_il = origin_h.cross(H_h);
    Vector3f l_ir = W_h.cross(WH_h);

    // compute line PQ
    Vector3f P_h(P.x,P.y,1);
    Vector3f Q_h(Q.x,Q.y,1);
    Vector3f l =  P_h.cross(Q_h);

    // compute intersections with all four lines
    Vector2f R_t = hom2euc(l.cross(l_it));
    Vector2f R_l = hom2euc(l.cross(l_il));
    Vector2f R_b = hom2euc(l.cross(l_ib)); 
    Vector2f R_r = hom2euc(l.cross(l_ir));

    // compute points within the image 
    MatrixXf R(4,2);
    R << R_t(0), R_t(1),
         R_l(0), R_l(1),
         R_b(0), R_b(1),
         R_r(0), R_r(1);
    Vector4i in = is_in_image(R);
    
    // // intersection points
    MatrixXf S(2,2);
    S << 0,0,
         0,0;

    VectorXi Sind(2);
         Sind << 0,0;

    // Sind = find(in); -> matlab 
    int cnt = 0;
    for(int i = 0; i < 4; i++){
      if(in(i) == 1){
        S.row(cnt) = R.row(i);
        Sind(cnt) = cnt; 
        cnt++;
      }
    }

    // angle PQ
    float Y = Q.y-P.y;
    float X = Q.x-P.x;
    float phi = wrapToPi(atan2(Y,X)+M_PI);
    
    //distance to P 
    MatrixXf _P(2,1);
    _P << P.x,
          P.y;
    Vector2f D = dist(S,_P);

    // points along the line
    MatrixXf S_(2,2);
    S_ << P.x + D(0) * cos(phi), P.y + D(0) * sin(phi),
          P.x + D(1) * cos(phi), P.y + D(1) * sin(phi); 

    // // distance to original points
    Vector2f DS = dist(S,S_);

    // matching points
    // [~, minds] = min(DS);
    int minds = 0;
    if(DS(0,0) < DS(1,0)) minds = 0;
    else minds = 1;
    side = Sind(minds);
    // Intersection of P with image boundaries
    P.x = S(minds,0);
    P.y = S(minds,1);

    // Intersection of Q with image boundaries
    // [~, maxds] = max(DS);
    int maxds = 0;
    if(DS(0,0) < DS(1,0)) maxds = 1;
    Q.x = S(maxds,0);
    Q.y = S(maxds,1);
  }
  //*************************************************************************************************
  Vector2f AgribotVS::hom2euc(Vector3f Mat){
    Vector2f _Mat;
    _Mat <<  Mat(0,0)/Mat(2,0),
            Mat(1,0)/Mat(2,0);
    return _Mat;
  }
  VectorXi AgribotVS::find(Eigen::Vector4i A){
    VectorXi idxs;
    for(int i=0; i<A.size(); i++)
        if(A(i))
            idxs << i;
    return idxs;
  }
  Vector4i AgribotVS::is_in_image(MatrixXf R){
    Vector4i _In(0, 0, 0, 0);
    for(int i = 0; i < 4; i++){
      if(R(i,0) >= 0 && R(i,0) <= width && R(i,1) >= 0 && R(i,1) <= height){
        _In(i) = 1;
      }
    }
    return _In;
  }
  MatrixXf AgribotVS::is_in_image_point(MatrixXf R){
    MatrixXf out_p(2,2);
    int cnt = 0;
    for(int i = 0; i < 4; i++){
      if(R(i,0) >= 0 && R(i,0) <= width && R(i,1) >= 0 && R(i,1) <= height){
        out_p(cnt,0) = R(i,0);
        out_p(cnt,1) = R(i,1);
        cnt++;
      }
    }
    return out_p;
  }
  //*************************************************************************************************
  Vector2f AgribotVS::dist(MatrixXf& A, MatrixXf& B){
    Vector2f dis;
    Vector2f tmp;
    //D = sqrt(sum((A-B).^2,2));
    for(unsigned int i = 0; i < A.cols(); i++){
        tmp << sqrt(pow((A(0,i) - B(0,0)),2)),
              sqrt(pow((A(1,i) - B(0,0)),2));
        dis << tmp;
    }
    return dis;
  }
  void AgribotVS::switch_cameras(int& cam_primary){
    if(cam_primary == 1)
        cam_primary=2;
    else{
       cam_primary =1;
    }
  }
  //*************************************************************************************************
  Point2f AgribotVS::camera2image(Point2f& xc){
    Point2f xi;
    xi.x =  xc.x - width/2;
    xi.y =  xc.y - height/2;
    return xi;
  }
  Point2f AgribotVS::camera2origin(Point2f& xc){
    Point2f xi;
    xi.x =  xc.x;
    xi.y =  height - xc.y;
    return xi;
  }
  Point2f AgribotVS::origin2camera(Point2f& xc){
    Point2f xi;
    xi.x =  xc.x;
    xi.y =  height - xc.y;
    return xi;
  }
  Point2f AgribotVS::origin2image(Point2f& xc){
    Point2f xi,xo;
    xi = origin2camera(xc);
    xo = camera2origin(xi);
    return xo;
  }
  Point2f AgribotVS::image2camera(Point2f& xc){
    Point2f xi;
    xi.x =  xc.x + width/2;
    xi.y =  xc.y + height/2;
    return xi;
  }
  Point2f AgribotVS::image2origin(Point2f& xc){
    Point2f xi,xo;
    xi = image2camera(xc);
    xo = camera2origin(xi);
    return xo;
  }
  //*************************************************************************************************
  void AgribotVS::initialize_neigbourhood(camera& I){
    cout << ex_Xc << " " << ex_Yc << endl;
    I.nh.Xc = ex_Xc;
    I.nh.Yc = ex_Yc;
    I.nh.L = nh_L;
    I.nh.H = nh_H;
    I.nh.offset = nh_offset;
    I.nh.shift = navigation_dir * I.nh.offset;
    cout << "***********initialize_neigbourhood***********" << endl;
  }
  void AgribotVS::shift_neighbourhood(camera& I, int shift_dir){
    // To Do:
    // compute using proper formula later
    int dX = shift_dir*I.nh.shift;
    cout << "###########shift_neighbourhood############" << endl;
    // new neighbourhood to select row
    cout << I.nh.Xc  << " " << I.nh.Xc + dX  << " " << I.nh.shift << "  " << shift_dir << endl;
    I.nh.Xc = I.nh.Xc + dX;
  }
  void AgribotVS::is_in_neigbourhood(camera& I){
    I.nh_points.clear();
    for(size_t i = 0; i < I.points.size(); i++){
      if(I.points[i].x > I.nh.Xc - I.nh.L/2 && I.points[i].x < I.nh.Xc + I.nh.L/2 
      && I.points[i].y > I.nh.Yc -I.nh.H/2 && I.points[i].y < I.nh.Yc + I.nh.H/2){
        I.nh_points.push_back(I.points[i]);
      }
    }
  }
  void AgribotVS::draw_neighbourhood(camera& I){
    Point2f P(I.nh.Xc,I.nh.Yc);
    int X = P.x - I.nh.L/2;
    int Y = P.y - I.nh.H/2;
    Rect RectangleToDraw3(X, Y, I.nh.L, I.nh.H);
    rectangle(I.image, RectangleToDraw3, Scalar(255, 204, 102), 3, 8, 0);
  }
  //*************************************************************************************************
  void AgribotVS::draw_features(camera& I, Vector3f Feature, cv::Scalar color){
    
    Point2f P_1(Feature(0),Feature(1));
    P_1 = image2camera(P_1);
    Point2f P_2(P_1.x +  100*cos(Feature(2) - M_PI/2),
                P_1.y + 100*sin(Feature(2) - M_PI/2));
    arrowedLine(I.image, P_1, P_2, color,3);
  }
  vector<Point2f> AgribotVS::Compute_nh_ex(camera& I){
    vector<Point2f> nh_ex;
    nh_ex.resize(2);
    int tmp_min_y = height;
    int tmp_max_y = 0;

    for(size_t i = 0; i < I.nh_points.size(); i++)
    {
      if(I.nh_points[i].y < tmp_min_y){
        tmp_min_y = I.nh_points[i].y;
        nh_ex[0] = I.nh_points[i];
      }
      if(I.nh_points[i].y > tmp_max_y){
        tmp_max_y = I.nh_points[i].y;
        nh_ex[1] = I.nh_points[i];
      }
    }
    return nh_ex;
  }
  float AgribotVS::compute_Theta(Point2f& P, Point2f& Q){
    // compute phi
    float Y = -Q.y+P.y;
    float X = Q.x-P.x;
    float phi = atan2(Y,X);

    // compute Theta
    float Theta = wrapToPi(M_PI/2 - phi);
    
    return Theta;
  }
  float AgribotVS::wrapToPi(float angle){
    while(angle < -M_PI && angle > M_PI){
      if(angle > M_PI){
        angle = angle - 2*M_PI;
      }else if(angle < -M_PI){
        angle = angle + 2*M_PI;
      }
    }
      return angle;
  }
  std::vector<double> AgribotVS::getEulerAngles(const nav_msgs::Odometry::ConstPtr& Pose) {
    std::vector<double> EulerAngles;
    EulerAngles.resize(3, 0);
    tf::Quaternion q(Pose->pose.pose.orientation.x, Pose->pose.pose.orientation.y,
                    Pose->pose.pose.orientation.z,
                    Pose->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(EulerAngles[0], EulerAngles[1], EulerAngles[2]);
    return EulerAngles;
  }
}  // namespace agribot_vs
