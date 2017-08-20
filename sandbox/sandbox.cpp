#include <opencv2/opencv.hpp>
using namespace cv;
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

class PathPlannerGrid{
  public:
    int goal_id;//id of april tag goal id
    int robot_id;//id of robot whose path needs to be planned
    int cell_size;//cell size in pixels
    vector<pt> path_points;
    vector<pair<int,int> > pixel_path_points;
    int total_points;
    int start_grid_x,start_grid_y;
    int goal_grid_x, goal_grid_y;
    vector<vector<int> > world_grid;grid size is assumed to be manueveurable by the robot
    PathPlannerGrid(int cs, int a, bool dr):goal_id(a),draw_path(dr),cell_size(cs){}
    void addPoint(double x,double y){
      if(total_points>=path_points.size())
        path_points.resize(100+path_points.size());add hundred points in one go
      path_points[total_points].x = x;
      path_points[total_points].y = y;
      total_points++;
    }
    void overlayGrid(vector<AprilTags::TagDetection> &detections,Mat &image){robot size from tagsize
      finds the obstacles, the robot and the goal grid coordinates and intialize world_grid
    }
    void findshortest(AprilInterfaceAndVideoCapture &testbed, Mat &image){find shortest traversal,populate path_points, draw if required
    }
};

int main(int argc, char **argv){
  Mat image;
  image = imread(argv[1]);
  namedWindow("what do you see?",WINDOW_NORMAL);
  imshow("what do you see?",image);
  cout<<"image type is "<<image.type()<<endl;
  Mat grayimage;
  cvtColor(image,grayimage,CV_BGR2GRAY);
  namedWindow("what a dog sees",WINDOW_NORMAL);
  imshow("what a dog sees",grayimage);
  Mat threshed;
  threshold(grayimage,threshed,100,255,0);
  namedWindow("what I want",WINDOW_NORMAL);
  imshow("what I want",threshed);
  waitKey(0);
  return 0;
}
