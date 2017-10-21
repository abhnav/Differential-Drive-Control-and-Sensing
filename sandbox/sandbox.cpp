#include "aprilvideointerface.h"
#include <unistd.h>
#include "pathplanners.h"
#include "controllers.h"
// For Arduino: serial port access class
#include "Serial.h"
using namespace std;
using namespace cv;

struct bot_config{
  PathPlannerGrid plan;
  PurePursuitController control;//constructor called, thus must have a default constructor with no arguments
  int id;
  robot_pose pose;
  //using intializer list allows intializing variables with non trivial contructor
  //assignment would not help if there is no default contructor with no arguments
  bot_config(int cx,int cy, int thresh,vector<vector<nd> > &tp, double a,double b,double c, int d,int e,int f,bool g):plan(PathPlannerGrid(cx,cy,thresh,tp)),control(PurePursuitController(a,b,c,d,e,f,g)){
    id = -1;//don't forget to set the id
    //below line would first call plan PathPlannerGrid constructor with no argument and then replace lhs variables with rhs ones
    //plan = PathPlannerGrid(cx,cy,thresh,tp);
  }
  void init(){
    plan.start_grid_x = plan.start_grid_y = -1;
    plan.robot_id = -1;
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
  int frame = 0;
  int first_iter = 1;
  double last_t = tic();
  const char *windowName = "What do you see?";
  cv::namedWindow(windowName,WINDOW_NORMAL);
  Serial s_transmit;
  if(testbed.m_arduino)
    s_transmit.open("/dev/ttyUSB0",9600);
  cv::Mat image;
  cv::Mat image_gray;
  //make sure that lookahead always contain atleast the next path point
  //if not then the next point to the closest would automatically become target
  //PurePursuitController controller(40.0,2.0,14.5,70,70,128,false);
  //PurePursuitController controller(20.0,2.0,14.5,70,70,128,true);
  //PathPlannerUser path_planner(&testbed);
  //setMouseCallback(windowName, path_planner.CallBackFunc, &path_planner);
  int robotCount;
  int max_robots = 3;
  int origin_id = 0;//always 0
  //tag id should also not go beyond max_robots
  vector<vector<nd> > tp;//a map that would be shared among all
  vector<bot_config> bots(max_robots,bot_config(60,60,100,tp,40.0,2.0,14.5,70,70,128,false));
  vector<PathPlannerGrid> planners(max_robots,PathPlannerGrid(tp));

  while (true){
    for(int i = 0;i<max_robots;i++){
      bots[i].init();
      bots[i].id = i;//0 is saved for origin
    }
    robotCount = 0;
    //testbed.m_cap >> image;
    image = imread("tagimage.jpg");

    testbed.processImage(image, image_gray);//tags extracted and stored in class variable
    int n = testbed.detections.size();
    for(int i = 0;i<n;i++){
      if(testbed.detections[i].id == origin_id){//plane extracted
        testbed.extractPlane(i);
        break;
      }
    }
    for(int i = 0;i<n;i++){
      bots[testbed.detections[i].id].plan.robot_id = i;
      if(testbed.detections[i].id != origin_id){//robot or goal
        if(robotCount>=10){
          cout<<"too many robots found"<<endl;
          break;
        }
        testbed.findRobotPose(i,bots[testbed.detections[i].id].pose);//i is the index in detections for which to find pose
      }
    }
    if(bots[0].plan.robot_id<0) continue;//can't find the origin to extract plane
    for(int i = 0;i<bots.size();i++){
      //for bot 0, the origin and robot index would be the same
      bots[i].plan.origin_id = bots[0].plan.robot_id;//set origin index of every path planner
      planners[i] = bots[i].plan;
    }

    //all robots must be detected(in frame) when overlay grid is called else some regions on which a robot is 
    //present(but not detected) would be considered an obstacle
    //no two robots must be present in the same grid cell(result is undefined)
    if(first_iter){
      first_iter = 0;
      bots[0].plan.overlayGrid(testbed.detections,image_gray);//overlay grid completely reintialize the grid, we have to call it once at the beginning only when all robots first seen simultaneously(the surrounding is assumed to be static) not every iteration
    }
    for(int i = 1;i<bots.size();i++)
      bots[i].plan.BSACoverageIncremental(testbed,bots[i].pose, 2.5,planners);

    //if(!path_planner.total_points){//no path algorithm ever run before, total_points become -1 if no path exists from pos to goal
      //path_planner.robot_id = tag_id_index_map[robot_id];
      //path_planner.goal_id = tag_id_index_map[goal_id];
      //path_planner.origin_id = tag_id_index_map[origin_id];
      //path_planner.overlayGrid(testbed.detections,image_gray);
      //if(path_planner.origin_id>=0 && path_planner.robot_id>=0){
        //path_planner.findCoverageGlobalNeighborPreference(testbed);
        //path_planner.findCoverageLocalNeighborPreference(testbed,robots[pose_id_index_map[robot_id]]);
        //path_planner.BSACoverage(testbed,robots[pose_id_index_map[robot_id]]);
        //if(path_planner.goal_id>=0)
          //path_planner.findshortest(testbed);
      //}
    //}

    pair<int,int> wheel_velocities;
    for(int i = 1;i<bots.size();i++){//0 is for origin
      wheel_velocities = bots[i].control.computeStimuli(bots[i].pose,bots[i].plan.path_points);
      if(testbed.m_arduino){
        s_transmit.print((uchar)(bots[i].id));
        s_transmit.print((uchar)(128+wheel_velocities.first));
        s_transmit.print((uchar)(128+wheel_velocities.second));
        cout<<"velocities sent "<<wheel_velocities.first<<" "<<wheel_velocities.second<<endl;
      }
    }
    if(testbed.m_draw){
      for(int i = 0;i<n;i++){
        testbed.detections[i].draw(image);
      }
      bots[origin_id].plan.drawGrid(image);
      for(int i = 1;i<bots.size();i++){
        bots[i].plan.drawPath(image);
      }
      //add a next point circle draw for visualisation
      //add a only shortest path invocation drawing function in pathplanners
      //correct next point by index to consider reach radius to determine the next point
      imshow(windowName,image);
    }
    // print out the frame rate at which image frames are being processed
    frame++;
    if (frame % 10 == 0) {
      double t = tic();
      cout << "  " << 10./(t-last_t) << " fps" << endl;
      last_t = t;
    }
    if (cv::waitKey(10) == 27) break;//until escape is pressed
  }
  return 0;
}
