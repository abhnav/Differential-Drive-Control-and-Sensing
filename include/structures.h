#ifndef STRUCT_H
#define STRUCT_H
#include <utility>
#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;
struct robot_pose{
  double x,y,omega;
  robot_pose():x(0),y(0),omega(0){}
};
//structure to store path points in world coordinates
struct pt{
  double x,y;
  pt(){}
  pt(double a,double b):x(a),y(b){}
};
struct nd{
  int tot;
  int blacks, whites;
  int tot_x, tot_y;//to calculate the middle pixel for the cell
  std::pair<int,int> parent;//parent in bfs, not used in global preference dfs(but parent is still set nevertheless), parent in local dfs, parent in BSA
  int steps;//steps in bfs(also used to indicate visited nodes), states in global preference dfs used in finding coverage, 0 = uncovered, 1 = 0th child, 2 = 1st child, 3 = 2nd child, 4 = 3rd child, 5 = all covered, visited in local dfs
  int wall_reference;//-1 no wall, 0 front wall, 1 right wall, 2 left wall, 3 back wall, used in BSA

  nd():tot(0),blacks(0),whites(0),tot_x(0),tot_y(0){
    wall_reference = parent.first = parent.second = -1;
    steps = 0;
  }
  void emptyCell(){
    tot = blacks = whites = tot_x = tot_y = steps = 0;
    parent.first = parent.second = wall_reference = -1;
  }
};
#endif