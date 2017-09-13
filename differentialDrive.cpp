#include "aprilvideointerface.h"
#include "pathplanners.h"
#include "controllers.h"
// For Arduino: locally defined serial port access class
#include "Serial.h"
using namespace std;
using namespace cv;

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
  double last_t = tic();
  const char *windowName = "What do you see?";
  cv::namedWindow(windowName,WINDOW_NORMAL);
  Serial s_transmit;
  if(testbed.m_arduino)
    s_transmit.open("/dev/ttyUSB0",9600);
  cv::Mat image;
  cv::Mat image_gray;
  PathPlannerGrid path_planner(60,60,100);
  //PathPlannerUser path_planner(&testbed);
  //setMouseCallback(windowName, path_planner.CallBackFunc, &path_planner);
  PurePursuitController controller(40.0,2.0,14.5,80,80,128,false);

  //make sure that lookahead always contain atleast the next path point
  //if not then the next point to the closest would automatically become target
  //PurePursuitController controller(20.0,2.0,14.5,70,70,128,true);
  int robot_id = 1,goal_id = 2,origin_id = 0;
  int robotCount;
  int max_robots = 10;
  vector<robot_pose> robots(max_robots);
  vector<int> tag_id_index_map(max_robots);//tag id should also not go beyond max_robots
  vector<int> pose_id_index_map(max_robots);
  while (true){
    for(int i = 0;i<max_robots;i++)
      tag_id_index_map[i] = pose_id_index_map[i] = -1;
    robotCount = 0;
    testbed.m_cap >> image;

    testbed.processImage(image, image_gray);//tags extracted and stored in class variable
    int n = testbed.detections.size();
    for(int i = 0;i<n;i++){
      if(testbed.detections[i].id == origin_id){//plane extracted
        testbed.extractPlane(i);
        break;
      }
    }
    for(int i = 0;i<n;i++){
      tag_id_index_map[testbed.detections[i].id] = i;
      if(testbed.detections[i].id != origin_id){//robot or goal
        if(robotCount>=10){
          cout<<"too many robots found"<<endl;
          break;
        }
        testbed.findRobotPose(i,robots[robotCount++]);//i is the index in detections for which to find pose
        pose_id_index_map[testbed.detections[i].id] = robotCount-1;
      }
    }
    if(path_planner.total_points == 0){
      path_planner.robot_id = tag_id_index_map[robot_id];
      path_planner.goal_id = tag_id_index_map[goal_id];
      path_planner.origin_id = tag_id_index_map[origin_id];
      path_planner.overlayGrid(testbed.detections,image_gray);
      //path_planner.findshortest(testbed);
      //path_planner.findCoverageGlobalNeighborPreference(testbed);
      if(pose_id_index_map[robot_id]>=0){
        //path_planner.findCoverageLocalNeighborPreference(testbed,robots[pose_id_index_map[robot_id]]);
        path_planner.BSACoverage(testbed,robots[pose_id_index_map[robot_id]]);
      }
    }

    pair<int,int> wheel_velocities = controller.computeStimuli(robots[pose_id_index_map[robot_id]],path_planner.path_points);
    if(testbed.m_arduino){
      s_transmit.print((uchar)robot_id);
      s_transmit.print((uchar)(128+wheel_velocities.first));
      s_transmit.print((uchar)(128+wheel_velocities.second));
      cout<<"velocities sent "<<wheel_velocities.first<<" "<<wheel_velocities.second<<endl;
    }
    if(testbed.m_draw){
      for(int i = 0;i<n;i++){
        testbed.detections[i].draw(image);
      }
      path_planner.drawGrid(image);
      path_planner.drawPath(image);
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
