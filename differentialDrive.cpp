#include <cmath>
#include <Eigen/Geometry>
#include <typeinfo>
#include <cstring>
#include <bits/stdc++.h>
#include <sys/time.h>
using namespace Eigen;
#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;
using namespace std;
struct robot_pose{
  double x,y,omega;
  robot_pose():x(0),y(0),omega(0){}
};
struct pt{
  double x,y;
  pt(){}
  pt(double a,double b):x(a),y(b){}
};

const string usage = "\n"
  "Usage:\n"
  "  apriltags_demo [OPTION...] [IMG1 [IMG2...]]\n"
  "\n"
  "Options:\n"
  "  -h  -?          Show help options\n"
  "  -a              Arduino (send tag ids over serial port)\n"
  "  -d              Disable graphics\n"
  "  -t              Timing of tag extraction\n"
  "  -C <bbxhh>      Tag family (default 36h11)\n"
  "  -D <id>         Video device ID (if multiple cameras present)\n"
  "  -F <fx>         Focal length in pixels\n"
  "  -W <width>      Image width (default 640, availability depends on camera)\n"
  "  -H <height>     Image height (default 480, availability depends on camera)\n"
  "  -S <size>       Tag size (square black frame) in meters\n"
  "  -E <exposure>   Manually set camera exposure (default auto; range 0-10000)\n"
  "  -G <gain>       Manually set camera gain (default auto; range 0-255)\n"
  "  -B <brightness> Manually set the camera brightness (default 128; range 0-255)\n"
  "\n";

const string intro = "\n"
    "April tags test code\n"
    "(C) 2012-2014 Massachusetts Institute of Technology\n"
    "Michael Kaess\n"
    "\n";
#ifndef __APPLE__
#define EXPOSURE_CONTROL // only works in Linux
#endif

#ifdef EXPOSURE_CONTROL
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <errno.h>
#endif
// OpenCV library for easy access to USB camera and drawing of images
#include "opencv2/opencv.hpp"
// April tags detector and various families that can be selected by command line option
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag36h11.h"
// Needed for getopt / command line options processing
#include <unistd.h>
extern int optind;
extern char *optarg;
using namespace cv;
// For Arduino: locally defined serial port access class
#include "Serial.h"
// utility function to provide current system time (used below in
// determining frame rate at which images are being processed)
double tic() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}
//Normalize angle to be within the interval [-pi,pi].
inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}
//Convert rotation matrix to Euler angles
void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
    yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
    roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}
//for determining opencv matrix type
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

//starts video capture and detects the april tags in the scene, also parses the command line options
class AprilInterfaceAndVideoCapture{
public:
  AprilTags::TagDetector* m_tagDetector;
  AprilTags::TagCodes m_tagCodes;
  bool m_draw; // draw image and April tag detections?
  bool m_arduino; // send tag detections to serial port?
  bool m_timing; // print timing information for each tag extraction call

  int m_width; // image size in pixels
  int m_height;
  double m_tagSize; // April tag side length in meters of square black frame
  double m_fx; // camera focal length in pixels
  double m_fy;
  double m_px; // camera principal point
  double m_py;

  int m_deviceId; // camera id (in case of multiple cameras)
  list<string> m_imgNames;

  cv::VideoCapture m_cap;

  int m_exposure;
  int m_gain;
  int m_brightness;

  Serial m_serial;

  Eigen::Vector3d planeOrigin;
  Eigen::Vector3d x_axis;
  Eigen::Vector3d y_axis;
  
  vector<AprilTags::TagDetection> detections;
  //remember to add a class var to specify video device number, currently being assumed at 0 in setupvideo
  AprilInterfaceAndVideoCapture() :
    // default settings, most can be modified through command line options (see below)
    m_tagDetector(NULL),
    m_tagCodes(AprilTags::tagCodes36h11),

    m_draw(true),
    m_arduino(false),
    m_timing(false),

    //below parameters are the most important
    //use a camera calibration technique to find out the below parameters
    //below parameters are found using opencv calibration example module in opencv installation 
    m_width(640),
    m_height(480),
    m_tagSize(0.135),
    m_fx(6.4205269897923586e+02),//focal length in pixels
    m_fy(6.4205269897923586e+02),
    m_px(m_width/2),
    m_py(m_height/2),

    m_exposure(-1),
    m_gain(-1),
    m_brightness(-1),

    m_deviceId(0){}

  // changing the tag family
  void setTagCodes(string s) {
    if (s=="16h5") {
      m_tagCodes = AprilTags::tagCodes16h5;
    } else if (s=="25h7") {
      m_tagCodes = AprilTags::tagCodes25h7;
    } else if (s=="25h9") {
      m_tagCodes = AprilTags::tagCodes25h9;
    } else if (s=="36h9") {
      m_tagCodes = AprilTags::tagCodes36h9;
    } else if (s=="36h11") {
      m_tagCodes = AprilTags::tagCodes36h11;
    } else {
      cout << "Invalid tag family specified" << endl;
      exit(1);
    }
  }
  // parse command line options to change default behavior
  void parseOptions(int argc, char* argv[]) {
    int c;
    while ((c = getopt(argc, argv, ":h?adtC:F:H:S:W:E:G:B:D:")) != -1) {
      // Each option character has to be in the string in getopt();
      // the first colon changes the error character from '?' to ':';
      // a colon after an option means that there is an extra
      // parameter to this option; 'W' is a reserved character
      switch (c) {
      case 'h':
      case '?':
        cout << intro;
        cout << usage;
        exit(0);
        break;
      case 'a':
        m_arduino = true;
        break;
      case 'd':
        m_draw = false;
        break;
      case 't':
        m_timing = true;
        break;
      case 'C':
        setTagCodes(optarg);
        break;
      case 'F':
        m_fx = atof(optarg);
        m_fy = m_fx;
        break;
      case 'H':
        m_height = atoi(optarg);
        m_py = m_height/2;
         break;
      case 'S':
        m_tagSize = atof(optarg);
        break;
      case 'W':
        m_width = atoi(optarg);
        m_px = m_width/2;
        break;
      case 'E':
#ifndef EXPOSURE_CONTROL
        cout << "Error: Exposure option (-E) not available" << endl;
        exit(1);
#endif
        m_exposure = atoi(optarg);
        break;
      case 'G':
#ifndef EXPOSURE_CONTROL
        cout << "Error: Gain option (-G) not available" << endl;
        exit(1);
#endif
        m_gain = atoi(optarg);
        break;
      case 'B':
#ifndef EXPOSURE_CONTROL
        cout << "Error: Brightness option (-B) not available" << endl;
        exit(1);
#endif
        m_brightness = atoi(optarg);
        break;
      case 'D':
        m_deviceId = atoi(optarg);
        break;
      case ':': // unknown option, from getopt
        cout << intro;
        cout << usage;
        exit(1);
        break;
      }
    }

    if (argc > optind) {
      for (int i=0; i<argc-optind; i++) {
        m_imgNames.push_back(argv[optind+i]);
      }
    }
  }

void setup(){
  m_tagDetector = new AprilTags::TagDetector(m_tagCodes);
}

void setupVideo(){
#ifdef EXPOSURE_CONTROL
    // manually setting camera exposure settings; OpenCV/v4l1 doesn't
    // support exposure control; so here we manually use v4l2 before
    // opening the device via OpenCV; confirmed to work with Logitech
    // C270; try exposure=20, gain=100, brightness=150
    string video_str = "/dev/video0";
    video_str[10] = '0' + m_deviceId;
    int device = v4l2_open(video_str.c_str(), O_RDWR | O_NONBLOCK);

    if (m_exposure >= 0) {
      // not sure why, but v4l2_set_control() does not work for
      // V4L2_CID_EXPOSURE_AUTO...
      struct v4l2_control c;
      c.id = V4L2_CID_EXPOSURE_AUTO;
      c.value = 1; // 1=manual, 3=auto; V4L2_EXPOSURE_AUTO fails...
      if (v4l2_ioctl(device, VIDIOC_S_CTRL, &c) != 0) {
        cout << "Failed to set... " << strerror(errno) << endl;
      }
      cout << "exposure: " << m_exposure << endl;
      v4l2_set_control(device, V4L2_CID_EXPOSURE_ABSOLUTE, m_exposure*6);
    }
    if (m_gain >= 0) {
      cout << "gain: " << m_gain << endl;
      v4l2_set_control(device, V4L2_CID_GAIN, m_gain*256);
    }
    if (m_brightness >= 0) {
      cout << "brightness: " << m_brightness << endl;
      v4l2_set_control(device, V4L2_CID_BRIGHTNESS, m_brightness*256);
    }
    v4l2_close(device);
#endif 

    // find and open a USB camera (built in laptop camera, web cam etc)
    m_cap = cv::VideoCapture(m_deviceId);
        if(!m_cap.isOpened()) {
      cerr << "ERROR: Can't find video device " << m_deviceId << "\n";
      exit(1);
    }
    m_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
    m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);
    cout << "Camera successfully opened (ignore error messages above...)" << endl;
    cout << "Actual resolution: "
         << m_cap.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
         << m_cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;

}
void pixelToWorld(double x,double y,double &xd,double &yd){
    x = x-m_px;
    y = y-m_py;
    Eigen::Vector3d tp(x/m_fx,y/m_fy,1);
    Eigen::Vector3d normal = x_axis.cross(y_axis);
    double d = normal.dot(planeOrigin);
    double tpd = normal.dot(tp);
    double zdash = d/tpd;
    tp = zdash*tp;
    tp = tp-planeOrigin;
    xd = tp.dot(x_axis);
    yd = tp.dot(y_axis);
}
//find the normal vector to the plane formed by the endpoints of tag
void findNormal(Eigen::Vector3d &trans, Eigen::Matrix3d &rot, Eigen::Vector3d &result){
    Eigen::Vector3d origin(0,0,0),xcoord(1,0,0),ycoord(0,1,0);
    origin = rot*origin+trans;
    xcoord = rot*xcoord+trans;
    ycoord = rot*ycoord+trans;
    xcoord = xcoord-origin;
    ycoord = ycoord-origin;
    result = xcoord.cross(ycoord);
}
void extractPlane(int ind){
    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    detections[ind].getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,translation, rotation);
    Eigen::Vector3d ori(0,0,0);
    Eigen::Vector3d xone(1,0,0);
    Eigen::Vector3d yone(0,1,0);
    planeOrigin = (rotation*ori) + translation;
    x_axis = (rotation*xone) + translation;
    x_axis = x_axis - planeOrigin;
    x_axis.normalize();//this step is important
    y_axis = (rotation*yone) + translation;
    y_axis = y_axis - planeOrigin;
    y_axis.normalize();
  }
//robot is assumed to be facing positive y direction of it's apriltag
void findRobotPose(int ind, robot_pose &rob){
    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    detections[ind].getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,translation, rotation);
    Eigen::Vector3d ori(0,0,0);
    ori = (rotation*ori)+translation;//ori is now the robot centre
    rob.x = ori.dot(x_axis);
    rob.y = ori.dot(y_axis);
    Eigen::Vector3d ycoord(0,1,0);
    ycoord = (rotation*ycoord)+translation;
    double tempx = ycoord.dot(x_axis), tempy = ycoord.dot(y_axis);
    tempx = tempx-rob.x; tempy = tempy-rob.y;
    rob.omega = atan2(tempy,tempx);
}
  void processImage(cv::Mat& image, cv::Mat& image_gray) {
    // alternative way is to grab, then retrieve; allows for
    // multiple grab when processing below frame rate - v4l keeps a
    // number of frames buffered, which can lead to significant lag
    //      m_cap.grab();
    //      m_cap.retrieve(image);
    // detect April tags (requires a gray scale image)
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);
    double t0;
    if (m_timing) {
      t0 = tic();
    }
    detections = m_tagDetector->extractTags(image_gray);
    if (m_timing) {
      double dt = tic()-t0;
      cout << "Extracting tags took " << dt << " seconds." << endl;
    }
    cout << detections.size() << " tags detected:" << endl;
  }

  // Load and process a single image
  void loadImages() {
    cv::Mat image;
    cv::Mat image_gray;
    for (list<string>::iterator it=m_imgNames.begin(); it!=m_imgNames.end(); it++) {
      image = cv::imread(*it); // load image with opencv
      processImage(image, image_gray);
      while (cv::waitKey(100) == -1) {}
    }
  }

// check the image container to find if video is to be processed or image
  bool isVideo() {
    return m_imgNames.empty();
  }
}; 
class PathPlannerUser{
  public:
    vector<pt> path_points;
    vector<pair<int,int> > pixel_path_points;
    int total_points;
    AprilInterfaceAndVideoCapture *testbed;
    PathPlannerUser(AprilInterfaceAndVideoCapture *tb):total_points(0),testbed(tb){}
    void addPoint(int px, int py, double x,double y){
      if(total_points>=path_points.size()){
        path_points.resize(100+path_points.size());//add hundred points in one go
        pixel_path_points.resize(100+pixel_path_points.size());
      }
      path_points[total_points].x = x;
      path_points[total_points].y = y;
      pixel_path_points[total_points].first = px;
      pixel_path_points[total_points].second = py;
      total_points++;
    }
    void CallBackFunc(int event, int x, int y){
      static int left_clicked = 0;
      static int x_pixel_previous = -1;
      static int y_pixel_previous = -1;
      //ignore EVENT_RABUTTONDOWN, EVENT_MBUTTONDOWN 
       if(event == EVENT_LBUTTONDOWN){
          left_clicked = (left_clicked+1)%2;
          if(left_clicked){
            path_points.clear();
            pixel_path_points.clear();
            total_points = 0;
          }
          x_pixel_previous = x;
          y_pixel_previous = y;
       }
       else if(event == EVENT_MOUSEMOVE && left_clicked){
         double xd, yd;
         testbed->pixelToWorld(x,y,xd,yd);
         addPoint(x,y,xd,yd);
         x_pixel_previous = x; 
         y_pixel_previous = y;
       }
    }
    void drawPath(Mat &image){
      for(int i = 0;i<total_points-1;i++){
        line(image,Point(pixel_path_points[i].first,pixel_path_points[i].second),Point(pixel_path_points[i+1].first,pixel_path_points[i+1].second),Scalar(0,0,255),2);
      }
    }
  static void CallBackFunc(int event, int x, int y, int flags, void* userdata){
    PathPlannerUser *ppu = reinterpret_cast<PathPlannerUser*>(userdata);
    ppu->CallBackFunc(event,x,y);
  }
};


struct nd{
  int tot;
  int blacks, whites;
  int tot_x, tot_y;//to calculate the middle pixel for the cell
  pair<int,int> parent;//-1,-1 indicates univisited
  int steps;//steps in bfs
  nd():tot(0),blacks(0),whites(0),tot_x(0),tot_y(0){
    parent.first = parent.second = -1;
    steps = 0;
  }
};

//the cell empty criteria is an adjustable parameter as well
class PathPlannerGrid{
  public:
    //the ids below are the indexes in the detections vector, not the actual tag ids
    int goal_id;
    int robot_id;
    int origin_id;
    int cell_size_x;//cell size in pixels
    int cell_size_y;
    int threshold_value;
    vector<pt> path_points;
    vector<pair<int,int> > pixel_path_points;
    int total_points;
    int start_grid_x,start_grid_y;
    int goal_grid_x, goal_grid_y;
    int rcells, ccells;
    vector<vector<nd> > world_grid;//grid size is assumed to be manueveurable by the robot

    PathPlannerGrid(int csx,int csy,int th):cell_size_x(csx),cell_size_y(csy),threshold_value(th),total_points(0){}

    void addPoint(int ind,int px, int py, double x,double y){
      path_points[ind].x = x;
      path_points[ind].y = y;
      pixel_path_points[ind].first = px;
      pixel_path_points[ind].second = py;
      total_points++;
    }

    void setRobotId(int a){robot_id = a;}
    void setGoalId(int a){goal_id = a;}
    void setOriginId(int a){origin_id = a;}

    bool isEmpty(int r,int c){//criteria based on which to decide whether cell is empty
      if(world_grid[r][c].blacks > world_grid[r][c].whites*0.2)//more than 20 percent
        return false;
      return true;
    }
    bool pixelIsInsideTag(int x,int y,pair<float,float> *p){
      for(int i = 0;i<4;i++){
        int j = (i+1)%4;
        if((x-p[j].first)*(p[j].second-p[i].second) - (y-p[j].second)*(p[j].first-p[i].first) >= 0)
          continue;
        return false;
      }
      return true;
    }
    //note the different use of r,c and x,y in the context of matrix and image respectively
    //check for obstacles but excludes the black pixels obtained from apriltags
    void overlayGrid(vector<AprilTags::TagDetection> &detections,Mat &grayImage){
      threshold(grayImage,grayImage,threshold_value,255,0);
      namedWindow("unprocessed",WINDOW_NORMAL);
      imshow("unprocessed",grayImage);
      start_grid_y = detections[robot_id].cxy.first/cell_size_x;
      start_grid_x = detections[robot_id].cxy.second/cell_size_y;
      goal_grid_y = detections[goal_id].cxy.first/cell_size_x;
      goal_grid_x = detections[goal_id].cxy.second/cell_size_y;
      int r = grayImage.rows, c = grayImage.cols;
      rcells = ceil((float)r/cell_size_y);
      ccells = ceil((float)c/cell_size_x);
      world_grid.resize(rcells);
      for(int i = 0;i<rcells;i++) world_grid[i].resize(ccells);
      for(int i = 0;i<r;i++){
        for(int j = 0;j<c;j++){
          int gr = i/cell_size_y, gc = j/cell_size_x;
          world_grid[gr][gc].tot++;
          if(grayImage.at<uint8_t>(i,j) == 255 || pixelIsInsideTag(j+1,i+1,detections[robot_id].p) || pixelIsInsideTag(j+1,i+1,detections[goal_id].p) || pixelIsInsideTag(j+1,i+1,detections[origin_id].p)){
            world_grid[gr][gc].whites++;
            grayImage.at<uint8_t>(i,j) = 255;
          }
          else{
            world_grid[gr][gc].blacks++;
            grayImage.at<uint8_t>(i,j) = 0;
          }
          world_grid[gr][gc].tot_x += j+1;//pixel values are indexed from 1
          world_grid[gr][gc].tot_y += i+1;
        }
      }
      for(int i = 0;i<r;i += cell_size_y)
        for(int j = 0;j<c;j++)
          grayImage.at<uint8_t>(i,j) = 0;
      for(int i = 0;i<c;i+=cell_size_x)
        for(int j = 0;j<r;j++)
          grayImage.at<uint8_t>(j,i) = 0;
      namedWindow("processed",WINDOW_NORMAL);
      imshow("processed",grayImage);
    }
    //find shortest traversal,populate path_points
    void findshortest(AprilInterfaceAndVideoCapture &testbed){
      queue<pair<int,int> > q;
      q.push(make_pair(start_grid_x,start_grid_y));
      world_grid[start_grid_x][start_grid_y].parent.first = rcells;//just to define parent of 1st node
      world_grid[start_grid_x][start_grid_y].parent.second = ccells;//just to define parent of 1st node
      vector<pair<int,int> > aj = {{-1,0},{0,1},{1,0},{0,-1}};
      int ngr,ngc;
      pair<int,int> t;
      while(!q.empty()){
        t = q.front();q.pop();
        if(t.first == goal_grid_x && t.second == goal_grid_y)
          break;
        for(int i = 0;i<4;i++){
          ngr = t.first+aj[i].first, ngc = t.second+aj[i].second;
          if(ngr>=rcells || ngr<0 || ngc>=ccells || ngc<0 || world_grid[ngr][ngc].parent.first>=0 || !isEmpty(ngr,ngc))
            continue;
          world_grid[ngr][ngc].parent.first = t.first;
          world_grid[ngr][ngc].parent.second = t.second;
          world_grid[ngr][ngc].steps = world_grid[t.first][t.second].steps + 1;
          q.push(make_pair(ngr,ngc));
        }
      }
      if(!( t.first == goal_grid_x && t.second == goal_grid_y )){
        cout<<"no path to reach destination"<<endl;
        total_points = -1;//dummy
        return;
      }
      total_points = 0;
      int cnt = world_grid[t.first][t.second].steps+1;
      pixel_path_points.resize(cnt);
      path_points.resize(cnt);
      for(int i = cnt-1;!(t.first == rcells && t.second == ccells);i--){
        int ax,ay;double bx,by;
        ax = world_grid[t.first][t.second].tot_x/world_grid[t.first][t.second].tot;
        ay = world_grid[t.first][t.second].tot_y/world_grid[t.first][t.second].tot;
        testbed.pixelToWorld(ax,ay,bx,by);
        addPoint(i,ax,ay,bx,by);
        t = world_grid[t.first][t.second].parent;
      }
    }
    void drawPath(Mat &image){
      for(int i = 0;i<total_points-1;i++){
        line(image,Point(pixel_path_points[i].first,pixel_path_points[i].second),Point(pixel_path_points[i+1].first,pixel_path_points[i+1].second),Scalar(0,0,255),2);
      }
    }
};


class PurePursuitController{
  public:
    double look_ahead_distance;
    double reach_radius;
    double axle_length;
    int linear_velocity;
    int inplace_turn_velocity;
    int max_velocity;
    double eps = 1e-9;
    double min_turn_radius;
    void calculateMinimumTurnRadius(){//finds turn radius in axle length scale
      min_turn_radius = (axle_length*(linear_velocity+max_velocity))/(2*(max_velocity-linear_velocity));
    }
    PurePursuitController(double a,double b,double c, int d,int e,int f):look_ahead_distance(a),reach_radius(b),axle_length(c),linear_velocity(d),inplace_turn_velocity(e),max_velocity(f){
      calculateMinimumTurnRadius();
    }
    double distance(double x1,double y1,double x2,double y2){
      return sqrt(pow(x1-x2,2) + pow(y1-y2,2));
    }
    pair<int,int> computeStimuli(robot_pose &rp,vector<pt> &path){
      int n = path.size();
      if(!n) return make_pair(0,0);
      double min = 1e15;
      int ind = -1;
      vector<double> distances(n);
      for(int i = 0;i<n;i++){
        if((distances[i]=distance(rp.x,rp.y,path[i].x,path[i].y))<min){
          min = distances[i];
          ind = i;
        }
        if(i == n-1 && distances[i]<=reach_radius)
          return make_pair(0,0);
      }
      int next_point = ind;
      for(int i = ind;i<n;i++){
        if(distances[i]<look_ahead_distance && distances[i]>distances[next_point])
          next_point = i;
      }
      Vector2d vec_translate(path[next_point].x-rp.x, path[next_point].y-rp.y);
      Rotation2D<double> rot(-(rp.omega-PI/2.0));
      Vector2d robot_relative_coords = rot*vec_translate;
      if(abs(robot_relative_coords(0))<eps)
        return make_pair(linear_velocity,linear_velocity);
      int flag_turn_left = 0;
      if(robot_relative_coords(0)<0){//2 quadrants to consider now
        flag_turn_left = 1;
        robot_relative_coords(0) *= -1;
      }
      if(robot_relative_coords(1)<0){
        if(flag_turn_left) return make_pair((-1)*inplace_turn_velocity,inplace_turn_velocity);
        else return make_pair(inplace_turn_velocity,(-1)*inplace_turn_velocity);
      }
      double radius_of_curvature = pow(distances[next_point],2)/(2.0*robot_relative_coords(0));
      if(radius_of_curvature<min_turn_radius){
        if(flag_turn_left) return make_pair((-1)*inplace_turn_velocity,inplace_turn_velocity);
        else return make_pair(inplace_turn_velocity,(-1)*inplace_turn_velocity);
      }
      int excess_turn = (2*axle_length*linear_velocity)/(2*radius_of_curvature-axle_length);
      if(flag_turn_left) return make_pair(linear_velocity,linear_velocity+excess_turn);
      else return make_pair(linear_velocity+excess_turn,linear_velocity);
    }
};

int main(int argc, char* argv[]) {
  AprilInterfaceAndVideoCapture testbed;
  testbed.parseOptions(argc, argv);
  testbed.setup();
  if (!testbed.isVideo()) {
    cout << "Processing image: option is not supported" << endl;
    testbed.loadImages();
    return 0;
  }
  cout << "Processing video" << endl;
  testbed.setupVideo();
  cv::Mat image;
  cv::Mat image_gray;
  vector<robot_pose> robots(10);
  PathPlannerGrid path_planner(10,10,100);
  //PathPlannerUser path_planner(&testbed);
  PurePursuitController controller(0.110,0.05,0.10,150,200,255);
  Serial s_transmit;
  s_transmit.open("/dev/ttyUSB0",9600);

  int frame = 0;
  double last_t = tic();
  const char *windowName = "What do you see?";
  cv::namedWindow(windowName,WINDOW_NORMAL);
  //setMouseCallback(windowName, path_planner.CallBackFunc, &path_planner);
  int robotCount;
  int robot_id = 1,robindex = -1,goal_id = 2,goalindex = -1,origin_id = 0, originindex = -1, robot_pose_index = -1;
  while (true){
    robotCount = 0;
    testbed.m_cap >> image;
    testbed.processImage(image, image_gray);//tags extracted and stored in class variable
    int n = testbed.detections.size();
    for(int i = 0;i<n;i++){
      if(testbed.detections[i].id == origin_id){//plane extracted
        originindex = i;
        testbed.extractPlane(i);
      }
      else if(testbed.detections[i].id == goal_id)
        goalindex = i;
      else{//it's a robot
        if(robotCount>=10){
          cout<<"too many robots found"<<endl;
          break;
        }
        testbed.findRobotPose(i,robots[robotCount++]);
        if(testbed.detections[i].id == robot_id){
          robot_pose_index = robotCount-1;
          robindex = i;
        }
      }
    }
    if(robindex>=0 && goalindex>=0 && originindex>=0){
      path_planner.setRobotId(robindex);
      path_planner.setOriginId(originindex);
      path_planner.setGoalId(goalindex);
      if(path_planner.total_points == 0){
        path_planner.overlayGrid(testbed.detections,image_gray);
        path_planner.findshortest(testbed);
      }
    }
    pair<int,int> wheel_velocities = controller.computeStimuli(robots[robot_pose_index],path_planner.path_points);
    if(testbed.m_arduino && (wheel_velocities.first!=0 || wheel_velocities.second!=0)){
      s_transmit.print((uchar)robot_id);
      s_transmit.print((uchar)wheel_velocities.first);
      s_transmit.print((uchar)wheel_velocities.second);
      cout<<"velocities sent "<<wheel_velocities.first<<" "<<wheel_velocities.second<<endl;
    }
    if(testbed.m_draw){
      for(int i = 0;i<n;i++){
        testbed.detections[i].draw(image);
      }
      path_planner.drawPath(image);
      imshow(windowName,image);
    }
    // print out the frame rate at which image frames are being processed
    frame++;
    if (frame % 10 == 0) {
      double t = tic();
      cout << "  " << 10./(t-last_t) << " fps" << endl;
      last_t = t;
    }
    if (cv::waitKey(1) >= 0) break;
  }
  return 0;
}
