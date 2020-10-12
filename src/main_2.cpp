#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Geometry"
#include "helpers.h"
#include "json.hpp"
#include <typeinfo>
#include "spline.h"
using namespace std;

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  tk::spline sx;
  tk::spline sy;
  tk::spline sh;
  tk::spline sh_p;
  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

sx.set_points(map_waypoints_s,map_waypoints_x);
sy.set_points(map_waypoints_s,map_waypoints_y);  
vector<double> map_waypoints_h;
vector<double> map_waypoints_perp_h;

int size=map_waypoints_x.size();

for (int i=0;i<size;i++){
        double value=atan2((map_waypoints_y[i]-map_waypoints_y[i-1]),(map_waypoints_x[i]-map_waypoints_x[i-1]));
	map_waypoints_h.push_back(value);
        map_waypoints_perp_h.push_back(value-pi()/2;)
	}


sh.set_points(map_waypoints_s,map_waypoints_h);
sh_p.set_points(map_waypoints_s,map_waypoints_perp_h);

  int lane = 1;
  int target_lane=1;
  double const ref_velocity = 49.0; //mph 
  double const max_vel=ref_velocity*0.44;//in m/s
  double ref_vel=max_vel; //reference velocity
  double max_acc=10.0;//m/s^2 =1g

  double max_jerk_x=10.0;//m/s^3
  double vel=ref_vel;//m/s
  double acc=0.0;//m/s^2
  double ref_dist=1000; 
  int turn=1;
h.onMessage([&max_vel,&max_acc,&acc,&ref_vel,&vel,&ref_dist,&target_lane,&max_jerk_x,&lane,&turn,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);
      if (s != "") {
        auto j = json::parse(s);        
        string event = j[0].get<string>();       
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          
         double s1=car_s;


 	vel=20;
   	int T=30/vel;
  

         vector<double> start_s={s1,vel,0};
         vector<double> end_s={s1+30,vel,0};

         vector<double> start_d={0,0,0};
         vector<double> end_d={lane,0,0}; 

     
         vector<double> s=JMT(start_s,end_s,T);
         vector<double> d=JMT(start_d,end_d,T);


         vector<double> next_x_vals;
         vector<double> next_y_vals;
	 vector<double> pts_s;
         vector<double> pts_d;
         vector<double> pts_h_perp;
	 

	 for (double t=0.0;t<=T;t+=0.02){      
            pts_d.push_back(s[0]+s[1]*t+s[2]*pow(t,2)+s[3]*pow(t,3)+s[4]*pow(t,4)+s[5]*pow(t,5));
            pts_s.push_back(d[0]+d[1]*t+d[2]*pow(t,2)+d[3]*pow(t,3)+d[4]*pow(t,4)+d[5]*pow(t,5)); 
            pts_h_perp.push_back(sh_p(pts_s));
	    }
  

          double size=pts_s.size();
          for (int i=0;i<=size;i++){
		double x= sx(pts_s[i]);
		double y= sy(pts_s[i]);
	        x=x+pts_d[i]*cos(pts_h_perp[i]);	
		y=y+pts_d[i]*sin(pts_h_perp[i]);
	        next_x_vals.push_back(x);
        	next_y_vals.push_back(y);	
          }
        
  
     
              
          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
