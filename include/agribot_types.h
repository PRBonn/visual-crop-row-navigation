/***************************************************************************************/
/* Paper: Visual-Servoing based Navigation for Monitoring Row-Crop Fields              */
/*    Alireza Ahmadi, Lorenzo Nardi, Nived Chebrolu, Chis McCool, Cyrill Stachniss     */
/*         All authors are with the University of Bonn, Germany                        */
/* maintainer: Alireza Ahmadi                                                          */
/*          (Alireza.Ahmadi@uni-bonn.de / http://alirezaahmadi.xyz)                    */
/***************************************************************************************/

#pragma once
#include <vector>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>

using namespace cv;
using namespace std;
using namespace Eigen;

namespace agribot_vs {
    
/**
 * @brief Neighbourhood type containing 
 * features of each Neighbourhood
 * 
 */
struct Neighbourhood {
  int Xc;
  int Yc;
  int L;            // Length (and Width) of the neighbourhood box
  int H;
  int offset;       // Offset in pixels to capture next row. (TO DO: Compute using rho and crop-row distance) 
  float shift;      // Offset in pixels including the direction
} ;


/**
 * @brief object of camera containing features and 
 * neighbourhood properties 
 */
struct camera{
  Mat image;
  vector<int> id;
  vector<int> nh_id;
  vector<Point2f> points;
  vector<Point2f> nh_points; 
  vector<vector<Point>> contours;
  vector<Vec4i> lines;
  // set camera Intrinsics
  int height;
  int width;
  int f_mm;                // focal length (in mm)
  int s_w;                 // sensor width (in mm)
  vector<Point2f> nh_ex;
  Neighbourhood nh;
} ;

}