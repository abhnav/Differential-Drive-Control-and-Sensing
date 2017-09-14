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
    std::vector<std::vector<nd> > world_grid;//grid size is assumed to be manueveurable by the robot
    //the following matrix is used to encode local preference based on current place and parent place, one is added to avoid negative array index
    std::pair<int,int> aj[3][3][4];

    PathPlannerGrid(int csx,int csy,int th):cell_size_x(csx),cell_size_y(csy),threshold_value(th),total_points(0),start_grid_x(-1),start_grid_y(-1),goal_grid_x(-1),goal_grid_y(-1),robot_id(-1),goal_id(-1),origin_id(-1){
      initializeLocalPreferenceMatrix();
    }
    PathPlannerGrid():total_points(0),start_grid_x(-1),start_grid_y(-1),goal_grid_x(-1),goal_grid_y(-1),robot_id(-1),goal_id(-1),origin_id(-1){
      initializeLocalPreferenceMatrix();
    }

    void initializeLocalPreferenceMatrix();
    //invert visitable and non visitable cells
    void gridInversion(const PathPlannerGrid &planner);
    void addPoint(int ind,int px, int py, double x,double y);
    //criteria based on which to decide whether cell is empty
    bool isEmpty(int r,int c);
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
