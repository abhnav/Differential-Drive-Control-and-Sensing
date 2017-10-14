#ifndef PLANNER_H
#define PLANNER_H
#include "aprilvideointerface.h"
#include <vector>
#include <utility>
#include <stack>
#include <cstring>
//the cell empty criteria is an adjustable parameter as well
class PathPlannerGrid{
  public:
    //the ids below are the indexes in the detections vector, not the actual tag ids
    static cv::RNG rng;
    cv::Scalar path_color;
    int goal_id;
    int robot_id;
    int origin_id;
    //note the different use of r,c and x,y in the context of matrix and image respectively
    int cell_size_x;//cell size in pixels
    int cell_size_y;
    int threshold_value;
    std::vector<pt> path_points;
    std::vector<pair<int,int> > pixel_path_points;
    int total_points;
    int start_grid_x,start_grid_y;
    int goal_grid_x, goal_grid_y;
    int rcells, ccells;
    std::vector<std::vector<nd> > &world_grid;//grid size is assumed to be manueveurable by the robot
    //the following matrix is used to encode local preference based on current place and parent place, one is added to avoid negative array index
    std::pair<int,int> aj[3][3][4];
    stack<pair<int,int> > sk;//stack is needed to remember all the previous points visited and backtrack, should be unique for every instance, used primarily by the incremental bsa

    PathPlannerGrid(int csx,int csy,int th,std::vector<std::vector<nd> > &wg):cell_size_x(csx),cell_size_y(csy),threshold_value(th),total_points(0),start_grid_x(-1),start_grid_y(-1),goal_grid_x(-1),goal_grid_y(-1),robot_id(-1),goal_id(-1),origin_id(-1),world_grid(wg){
      initializeLocalPreferenceMatrix();
      path_color = cv::Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
    }
    PathPlannerGrid(std::vector<std::vector<nd> > &wg):total_points(0),start_grid_x(-1),start_grid_y(-1),goal_grid_x(-1),goal_grid_y(-1),robot_id(-1),goal_id(-1),origin_id(-1),world_grid(wg){
      initializeLocalPreferenceMatrix();
      path_color = cv::Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
    }
    PathPlannerGrid& operator=(const PathPlannerGrid& pt){
      path_color = pt.path_color;
      goal_id = pt.goal_id;
      robot_id = pt.robot_id;
      origin_id = pt.origin_id;
      cell_size_x = pt.cell_size_x;
      cell_size_y = pt.cell_size_y;
      threshold_value = pt.threshold_value;
      path_points = pt.path_points;
      pixel_path_points = pt.pixel_path_points;
      total_points = pt.total_points;
      start_grid_x = pt.start_grid_x,start_grid_y = pt.start_grid_y;
      goal_grid_x = pt.goal_grid_x, goal_grid_y = pt.goal_grid_y;
      rcells = pt.rcells, ccells = pt.ccells;
      world_grid = pt.world_grid;
      //no need to copy aj as all are the same
      sk = pt.sk;
      return *this;
    }
    double distance(double x1,double y1,double x2,double y2);
    void shareMap(const PathPlannerGrid &planner);
    void initializeLocalPreferenceMatrix();
    //invert visitable and non visitable cells
    void gridInversion(const PathPlannerGrid &planner,int rid);
    void addPoint(int ind,int px, int py, double x,double y);
    //criteria based on which to decide whether cell is empty
    bool isEmpty(int r,int c);
    bool isFellowAgent(int x,int y,std::vector<AprilTags::TagDetection> &detections);
    bool pixelIsInsideTag(int x,int y,std::vector<AprilTags::TagDetection> &detections,int ind);
    int setRobotCellCoordinates(std::vector<AprilTags::TagDetection> &detections);
    int setGoalCellCoordinates(std::vector<AprilTags::TagDetection> &detections);
    void drawGrid(cv::Mat &image);
    //image rows and columns are provided
    void initializeGrid(int r,int c);
    //check for obstacles but excludes the black pixels obtained from apriltags
    void overlayGrid(std::vector<AprilTags::TagDetection> &detections,cv::Mat &grayImage);
    //find shortest traversal,populate path_points
    void findshortest(AprilInterfaceAndVideoCapture &testbed);
    std::pair<int,int> setParentUsingOrientation(robot_pose &ps);
    void addGridCellToPath(int r,int c,AprilInterfaceAndVideoCapture &testbed);
    bool isBlocked(int ngr, int ngc);
    int getWallReference(int r,int c,int pr, int pc);
    void addBacktrackPointToStackAndPath(std::stack<std::pair<int,int> > &sk,std::vector<std::pair<int,int> > &incumbent_cells,int &ic_no,int ngr, int ngc,std::pair<int,int> &t,AprilInterfaceAndVideoCapture &testbed);
    void BSACoverage(AprilInterfaceAndVideoCapture &testbed,robot_pose &ps);
    int backtrackSimulateBid(std::pair<int,int> target,AprilInterfaceAndVideoCapture &testbed);
    void BSACoverageIncremental(AprilInterfaceAndVideoCapture &testbed, robot_pose &ps,double reach_distance,vector<PathPlannerGrid> &bots);
    void findCoverageLocalNeighborPreference(AprilInterfaceAndVideoCapture &testbed,robot_pose &ps);
    void findCoverageGlobalNeighborPreference(AprilInterfaceAndVideoCapture &testbed);
    void drawPath(cv::Mat &image);
};

class PathPlannerUser{
  public:
    std::vector<pt> path_points;
    std::vector<std::pair<int,int> > pixel_path_points;
    int total_points;
    AprilInterfaceAndVideoCapture *testbed;
    PathPlannerUser(AprilInterfaceAndVideoCapture *tb):total_points(0),testbed(tb){}
    void addPoint(int px, int py, double x,double y);
    void CallBackFunc(int event, int x, int y);
    void drawPath(cv::Mat &image);
    static void CallBackFunc(int event, int x, int y, int flags, void* userdata){
      PathPlannerUser *ppu = reinterpret_cast<PathPlannerUser*>(userdata);
      ppu->CallBackFunc(event,x,y);
    }
};

#endif
