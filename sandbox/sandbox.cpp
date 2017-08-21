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

    PathPlannerGrid(int csx,int csy, int a,int b,int th):robot_id(a),goal_id(b),cell_size_x(csx),cell_size_y(csy),threshold_value(th){}

    void addPoint(int ind,int px, int py, double x,double y){
      path_points[ind].x = x;
      path_points[ind].y = y;
      pixel_path_points[ind].first = px;
      pixel_path_points[ind].second = py;
      total_points++;
    }

    bool isEmpty(int r,int c){//criteria based on which to decide whether cell is empty
      if(world_grid[r][c].blacks > world_grid[r][c].whites*0.2)//more than 20 percent
        return false;
      return true;
    }
    bool pixelIsInsideTag(int x,int y,vector<pair<float,float> > &p){
      for(int i = 0;i<4;i++){
        int j = (i+1)%4;
        if((x-p[j].first)*(p[i].second-p[j].second) + (y-p[j].second)*(p[j].first-p[i].first) >= 0)
          continue;
        return false;
      }
      return true;
    }
    //note the different use of r,c and x,y in the context of matrix and image respectively
    //check for obstacles but excludes the black pixels obtained from apriltags
    void overlayGrid(vector<AprilTags::TagDetection> &detections,Mat &grayImage){
      threshold(grayImage,grayImage,threshold_value,255,0);
      start_grid_x = detections[robot_id].cxy.first/cell_size_x;
      start_grid_y = detections[robot_id].cxy.second/cell_size_y;
      goal_grid_x = detections[goal_id].cxy.first/cell_size_x;
      goal_grid_y = detections[goal_id].cxy.second/cell_size_y;
      int r = image.rows, c = image.cols;
      rcells = ceil((float)r/cell_size_y);
      ccells = ceil((float)c/cell_size_x);
      world_grid.resize(rcells);
      for(int i = 0;i<rcells;i++) world_grid[i].resize(ccells);
      for(int i = 0;i<r;i++){
        for(int j = 0;j<c;j++){
          int gr = i/cell_size_y, gc = j/cell_size_x;
          world_grid[gr][gc].tot++;
          if(grayImage[i][j] == 255 || pixelIsInsideTag(i+1,j+1,detections[robot_id].p) || pixelIsInsideTag(i+1,j+1,detections[goal_id].p)) world_grid[gr][gc].whites++;
          else world_grid[gr][gc].blacks++;
          world_grid[gr][gc].tot_x += j+1;//pixel values are indexed from 1
          world_grid[gr][gc].tot_y += i+1;
        }
      }
    }
    //find shortest traversal,populate path_points
    void findshortest(AprilInterfaceAndVideoCapture &testbed){
      queue<pair<int,int> > q;
      q.push(make_pair(start_grid_x,start_grid_y));
      world_grid[start_grid_x][start_grid_y].parent.first = rcells;//just to define parent of 1st node
      world_grid[start_grid_x][start_grid_y].parent.second = ccells;//just to define parent of 1st node
      vector<pair<int> > aj = {{-1,0},{0,1},{1,0},{0,-1}};
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
