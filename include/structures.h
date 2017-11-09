#ifndef STRUCT_H
#define STRUCT_H
#include <utility>
#include <stack>
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
  int r_id;//robot id that covered the given cell, starts from 0 and up

  nd():tot(0),blacks(0),whites(0),tot_x(0),tot_y(0){
    r_id = wall_reference = parent.first = parent.second = -1;
    steps = 0;
  }
  void emptyCell(){
    tot = blacks = whites = tot_x = tot_y = steps = 0;
    parent.first = parent.second = wall_reference = r_id = -1;
  }
};

struct bt{
  //the bt point might not remain valid, so you must check coverage for next_p in world grid before using it
  std::pair<int,int> parent;
  std::pair<int,int> next_p;
  std::stack<std::pair<int,int> > stack_state;
  int manhattan_distance;//distance of robot from this point's parent(returning distance)
  bool valid;
  bt(){valid = true;}
  bt(int pr,int pc, int r, int c, std::stack<std::pair<int,int> > sk){
    parent.first = pr, parent.second = pc, next_p.first = r, next_p.second = c;
    stack_state = sk;
    valid = true;
  }
};
#endif
