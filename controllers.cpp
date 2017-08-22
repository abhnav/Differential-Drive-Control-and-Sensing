struct robot_pose{
  double x,y,omega;
  robot_pose():x(0),y(0),omega(0){}
};
struct pt{
  double x,y;
  pt(){}
  pt(double a,double b):x(a),y(b){}
};
class PurePursuitController{
  public:
    double look_ahead_distance;
    double reach_radius;
    double axle_length;
    int linear_velocity;
    int inplace_turn_velocity;
    int max_velocity;
    double min_turn_radius;
    void calculateMinimumTurnRadius(){//finds turn radius in axle length scale
      min_turn_radius = (axle_length*(linear_velocity+max_velocity))/(2*(max_velocity-linear_velocity));
    }
    PurePursuitController(double a,double b,double c, int d,int e,int f):look_ahead_distance(a),reach_radius(b),axle_length(c),linear_velocity(d),inplace_turn_velocity(e),max_velocity(f){
      calculateMinimumTurnRadius();
    }
    double distance(double x1,double y2,double x2,double y2){
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
      vector2d vec_translate(path[next_point].x-rp.x, path[next_point].y-rp.y);
      Rotation2D<double> rot(-(rp.omega-PI/2.0));
      vector2d robot_relative_coords = rot*vec_translate;
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
