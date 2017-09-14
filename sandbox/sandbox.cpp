#include <opencv2/opencv.hpp>
#include <bits/stdc++.h>
#include <Eigen/Geometry>
using namespace cv;
using namespace Eigen;
using namespace std;

string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}
int main(int argc, char **argv){
  Rotation2D<float> rot(-M_PI/2);
  Vector2f v(1,0);
  v = rot*v;
  cout<<v<<endl;
  cout<<"what the fuk"<<endl;
  return 0;
}
