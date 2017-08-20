struct pt{
  double x,y;
  pt(){}
  pt(double a,double b):x(a),y(b){}
};
class PathPlannerUser{
  public:
    vector<pt> path_points;
    int total_points;
    bool draw_path;
    cv::Mat *image;//image is optional because our points are obtained via call back function
    AprilInterfaceAndVideoCapture *testbed;
    PathPlannerUser(AprilInterfaceAndVideoCapture *tb):total_points(0),draw_path(false),image(NULL),testbed(tb){}
    PathPlannerUser(bool draw, Mat *im,AprilInterfaceAndVideoCapture *tb):total_points(0),draw_path(draw),image(im),testbed(tb){}
    void addPoint(double x,double y){
      if(total_points>=path_points.size())
        path_points.resize(100+path_points.size());//add hundred points in one go
      path_points[total_points].x = x;
      path_points[total_points].y = y;
      total_points++;
    }
  void CallBackFunc(int event, int x, int y, int flags, void* userdata){
    static int left_clicked = 0;
    static int x_pixel_previous = -1;
    static int y_pixel_previous = -1;
    //ignore EVENT_RABUTTONDOWN, EVENT_MBUTTONDOWN 
     if(event == EVENT_LBUTTONDOWN){
        left_clicked = (left_clicked+1)%2;
        x_pixel_previous = x;
        y_pixel_previous = y;
     }
     else if(event == EVENT_MOUSEMOVE && left_clicked){
       if(draw_path)
          line(*image, Point(x,y), Point(x_pixel_previous, y_pixel_previous), Scalar(0,0,255), 2);
       double xd, yd;
       testbed->pixelToWorld(x,y,xd,yd);
       addPoint(xd,yd);
       x_pixel_previous = x; 
       y_pixel_previous = y;
     }
  }
};

