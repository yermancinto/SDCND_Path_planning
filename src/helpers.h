#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// for convenience
using std::string;
using std::vector;
using namespace std;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y,heading};
}

bool check_lane(int lane,double vehicle_speed,vector<vector<double>> sensor_fusion,double car_s,int prev_size){
   if ((lane>2)||(lane<0)){ //out of the limits of the road
       return false;
       }
   else{
       for (int i=0; i<sensor_fusion.size();i++){ 

       float d=sensor_fusion[i][6];
       double vx=sensor_fusion[i][3]; // in m/s
       double vy=sensor_fusion[i][4]; // in m/s
       double spd_front=sqrt(vx*vx+vy*vy); //calculate vehicle speed
       double check_car_s=sensor_fusion[i][5]+prev_size*0.02*spd_front;
       double gap=check_car_s-(car_s+prev_size*0.02*vehicle_speed);//+prev_size*0.02*vehicle_speed;
       
	if((d>(2+4*lane-2)) && (d<(2+4*lane+2))   ){//there is a vehicle in the lane we want to change to
          	if ((gap<10) && (gap>-10)){//there is no space 
             	std::cout<<"no gap:"<<gap<<"in lane"<<lane<<endl;
             	return false;
             	}
          //if ((gap<=-10)&&(gap>-20)&&(vehicle_speed<spd_front)){
          //   return false;
          //   }
          //if ((gap>=10)&&(gap<20)&&(vehicle_speed>spd_front)){
          //   return false;
          //   }
          }

       }
//std::cout<<"enough gap in lane"<<lane<<endl;
       return true;
   }

   }
int look_ahead(vector<vector<double>> sensor_fusion,double car_s){
  double look_0=100;
  double look_1=100;
  double look_2=100;
    
  for (int i=0; i<sensor_fusion.size();i++){ 
     double check_car_s=sensor_fusion[i][5];
     double d=sensor_fusion[i][6];
     double gap=check_car_s-car_s;
     if ((d>0)&&(d<4)&&(gap<100)&&(gap>0)){//vehicle ahead in lane 0
       if (gap<look_0){
         look_0=gap;
         }
       }
     else if ((d>4)&&(d<8)&&(gap<100)&&(gap>0)){//vehicle ahead in lane 1
        if (gap<look_1){
         look_1=gap;
         }
       }
     else if ((d>8)&&(d<12)&&(gap<100)&&(gap>0)){//vehicle ahead in lane 2
        if (gap<look_2){
         look_2=gap;
         }
       }
     }
   if((look_0>look_1)&&(look_0>look_2)){
     return 0;
     }
   if((look_1>look_0)&&(look_1>look_2)){
     return 1;
     }
   if((look_2>look_0)&&(look_2>look_1)){
     return 2;
     }
 }


double MAF(vector<double> vector, int values){  //moving average filter
    int vector_size=vector.size();
    double Mean=0;
    if (vector_size<values){
        for(int i=0;i<=vector_size; i++){
           Mean+=vector[i];
           }
         Mean=Mean/vector_size; 
        }
    else{
         for(int i=0;i<=values; i++){
            Mean+=vector[i];
            }
         Mean=Mean/values; 
         }
    return Mean;
    }






vector<double> JMT(vector<double> &start, vector<double> &end, double T) {
  MatrixXd A = MatrixXd(3, 3);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
       3*T*T, 4*T*T*T,5*T*T*T*T,
       6*T, 12*T*T, 20*T*T*T;
  MatrixXd B = MatrixXd(3,1);     
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
       end[1]-(start[1]+start[2]*T),
       end[2]-start[2];
  MatrixXd Ai = A.inverse();
  MatrixXd C = Ai*B;
  vector <double> result = {start[0], start[1], .5*start[2]};
  for(int i = 0; i < C.size(); ++i) {
    result.push_back(C.data()[i]);
    }
  return result;
}
  
 

#endif  // HELPERS_H
