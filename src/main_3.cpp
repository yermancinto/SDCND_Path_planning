#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Geometry"
#include "cppfsm-master/fsm.h"   //FSM library
#include "helpers.h"
#include "json.hpp"
#include <typeinfo>
#include "spline.h"
#include "trajectory_planner.cpp"

#include <math.h>
using namespace std;

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


enum States {KL,FV,L_LCH,R_LCH};
enum Triggers {Clear,Vehicle_ahead};
using F=FSM::Fsm<States,States::KL,Triggers>;
F fsm;
const char * StateNames[]={"Keep Lane","Follow vehicle","Change to left lane","Change to right lane"};
void dbg_fsm (States from_state,States to_state,Triggers trigger){
     if (from_state != to_state){
          std::cout<<"State changed to:"<<StateNames[to_state]<<"\n";
          }
     }


typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::milliseconds ms;

int main() {
  uWS::Hub h;
/**
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
	double value;	
	if (i==0){
		value=atan2((map_waypoints_y[i]-map_waypoints_y[map_waypoints_y.size()-1]),(map_waypoints_x[i]-map_waypoints_x[map_waypoints_y.size()]));
		}
	else{
        	value=atan2((map_waypoints_y[i]-map_waypoints_y[i-1]),(map_waypoints_x[i]-map_waypoints_x[i-1]));
		}	
	map_waypoints_h.push_back(value);
        map_waypoints_perp_h.push_back(value-pi()/2);
	}


sh.set_points(map_waypoints_s,map_waypoints_h);
sh_p.set_points(map_waypoints_s,map_waypoints_perp_h);
*/
  double lane = 1.0;
  int target_lane=1;
  double const ref_velocity = 49.0; //mph 
  double const max_vel=ref_velocity*0.44;//in m/s
  double ref_vel=max_vel; //reference velocity
  double max_acc=10.0;//m/s^2 =1g

  double max_jerk_x=10.0;//m/s^3
  double vel=ref_vel;//m/s
  double acc=0.0;//m/s^2
  double spd_front;
  double ref_dist=1000; 
  bool RIGHT;
  bool LEFT;



  Trajectory trajectory;


trajectory.load_map();

  //prev_trajectory.set_initial_state({0,0,0},{2+4*lane,0,0});
  //prev_trajectory.set_final_state({30,0,0},{2+4*lane,0,0});
  

 

std::vector<F::Trans> transitions = 
{
//From State,    To State,         Trigger,                  Guard,                   action
  {States::KL,    States::KL,       Triggers::Clear        ,  [&]{return true;}     ,[&]{ref_vel=max_vel;}   },
  {States::KL,    States::L_LCH,    Triggers::Vehicle_ahead,  [&]{return (LEFT  && (target_lane-lane<0));}     ,[&]{lane=lane-1;}       },
  {States::KL,    States::R_LCH,    Triggers::Vehicle_ahead,  [&]{return (RIGHT && (target_lane-lane>0));}    ,[&]{lane=lane+1;}       },
  {States::KL,    States::FV   ,    Triggers::Vehicle_ahead,  [&]{return true;}     ,[&]{ref_vel=spd_front;} },//!!!!

  {States::L_LCH, States::KL,       Triggers::Clear        ,  [&]{return true;}     ,[&]{ref_vel=max_vel;}   },
  {States::L_LCH, States::FV,       Triggers::Vehicle_ahead,  [&]{return true;}     ,[&]{ref_vel=spd_front;} },//!!!!

  {States::R_LCH, States::KL,       Triggers::Clear        ,  [&]{return true;}     ,[&]{ref_vel=max_vel;}   },
  {States::R_LCH, States::FV,       Triggers::Vehicle_ahead,  [&]{return true;}     ,[&]{ref_vel=spd_front;} },//!!!!

  {States::FV,    States::L_LCH,    Triggers::Vehicle_ahead,  [&]{return (LEFT  && (target_lane-lane<0));}     ,[&]{lane=lane-1;}       },
  {States::FV,    States::R_LCH,    Triggers::Vehicle_ahead,  [&]{return (RIGHT && (target_lane-lane>0));}    ,[&]{lane=lane+1;}       },
  {States::FV,    States::KL   ,    Triggers::Clear        ,  [&]{return true;}     ,[&]{ref_vel=max_vel;}   },
  {States::FV,    States::FV   ,    Triggers::Vehicle_ahead,  [&]{return true;}     ,[&]{ref_vel=spd_front;} },//!!!!
  };

   

fsm.add_transitions(transitions);
fsm.add_debug_fn(dbg_fsm);



h.onMessage([&max_vel,&ref_vel,&vel,&ref_dist,&target_lane,&lane,&trajectory,&fsm,&LEFT,&RIGHT]
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
          
          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          int prev_size=previous_path_x.size();
        
         //************************************
         //Check the environment of our vehicle
         //************************************
cout<<"car_s:"<<car_s<<endl;
 	  double spd_front=max_vel;
          ref_dist=50; //front detection distance in meters
          for (int i=0; i<sensor_fusion.size();i++){ 
            float d=sensor_fusion[i][6]; 
            if (d>(2+4*lane-2)&& d<(2+4*lane+2)){ //if another vehicle in our lane
              double check_car_s=sensor_fusion[i][5];
              //double sensor_fusion[i][0];//vehicle id
              double distance=check_car_s-car_s; //distance to the vehicle ahead
              if ((distance<ref_dist)&&(distance>0)){ //vehicle ahead at less than 50 meters
                ref_dist=distance;                
                double vx=sensor_fusion[i][3]; // in m/s
                double vy=sensor_fusion[i][4]; // in m/s
                spd_front=sqrt(vx*vx+vy*vy); //speed of the vehicle ahead                    
                }
            }  
          }
cout<<ref_dist<<endl;
cout<<ref_vel<<endl;
      if (ref_dist<50){ // If there is a vehicle ahead, check left and right lanes
                RIGHT=check_lane(lane+1,car_speed,sensor_fusion,car_s,prev_size);//check right lane
                LEFT=check_lane(lane-1,car_speed,sensor_fusion,car_s,prev_size);//check right lane
                }

         target_lane=look_ahead(sensor_fusion,car_s);
	//********************************************************
	//Execute the FSM as a function of the ref_distance*******
	//********************************************************
	 if (ref_dist<50){
	   fsm.execute(Triggers::Vehicle_ahead);
	   }
	 else{
	   fsm.execute(Triggers::Clear);
	   }
	//********************************************************
	//********************************************************

//*****************************************************************************************
          //LOOP TO SET THE REFERENCE SPEED**********************************************************
          //*****************************************************************************************
      //std::cout <<ref_dist<<endl; 
/**
          if (ref_dist<50){           
                 if (spd_front>max_vel){
                    ref_vel=max_vel;
                    }
                 else{
                    ref_vel=spd_front;
                    }
                 }
           else{
                 ref_vel=max_vel;
                 }

*/


/*
double total_acc;
          

  	if (vel<ref_vel){
      		double req_acc=(ref_vel-vel)/0.02;//required acceleration
              	double jerk_x=(req_acc-acc_x)/0.02;  
              	if (jerk_x>max_jerk_x){
          		acc_x+=max_jerk_x*0.02;
                   	}
              	else{
          		acc_x=req_acc;
                  	}
                total_acc=sqrt(pow(acc_x,2)+pow(acc_y,2));
              	//if (acc_x>max_acc){
                if (total_acc>max_acc){
          		acc_x=sqrt(pow(total_acc,2)-pow(acc_y,2));
                  	}
              	vel+=acc_x*0.02;  
              	}

 	if (vel>ref_vel){   
		double req_acc=(ref_vel-vel)/0.02;//required deceleration (-)
           	double jerk_x=abs((req_acc-acc_x)/0.02);  //negative value
           	if  (jerk_x>max_jerk_x){
             		acc_x-=max_jerk_x*0.02;
     			} 
           	else{
             		acc_x=req_acc;//negative value
             		}
           	if (abs(acc_x)>max_acc){
              		acc_x=-max_acc;
              	} 
           	vel+=acc_x*0.02;  
           	}

*/
cout<<"trajectory loop:"<<endl;

          if (prev_size>0){
		
               
	  //set initial values for the sleep()next path...
            	car_s=end_path_s;                            //need to know the speed at the end_path_s position
		vector <double> derivatives=trajectory.derivatives(end_path_s);
            	double car_s_dot=derivatives[0];       //s velocity at the end_path_s position
		double car_s_dot_dot=derivatives[1];   //s velocity at the end_path_s position

	   	double car_d_dot=derivatives[2];//s acceleration at the end_path_s position
		double car_d_dot_dot=derivatives[3];
		cout<<"derivatives initial state:"<<endl;

  	        cout<<car_s_dot<<endl;
		cout<<car_s_dot_dot<<endl;
		cout<<car_d_dot<<endl;
		cout<<car_d_dot_dot<<endl;

                //initial state of out next trajectory:
                 vector<double> i_s={end_path_s,car_s_dot,car_s_dot_dot};
                 vector<double> i_d={end_path_d,car_d_dot,car_d_dot_dot};
                //final state of our next trajectory:
                 vector<double> f_s={end_path_s+ref_dist,ref_vel,0.0};
                 vector<double> f_d={2+4*lane,0.0,0.0};

	        trajectory.set_initial_state(i_s,i_d);
	        trajectory.set_final_state(f_s,f_d);


		}
	else{ 
                cout<<"no previous trajectory:"<<endl;
             	 cout<<"car_s:"<<car_s<<endl;
                 vector<double> i_s={car_s,0.0,0.0};//initial s state 
		 vector<double> i_d={car_d,0.0,0.0};//initial d state 
                 vector<double> f_s={car_s+ref_dist,ref_vel,0};//final s state
                 vector<double> f_d={2+4*lane,0.0,0.0};//final d state



	        trajectory.set_initial_state(i_s,i_d);
	        trajectory.set_final_state(f_s,f_d);
		
         }

         trajectory.get_trajectory();

          vector<double> next_x_vals;
          vector<double> next_y_vals;
          //start with all the points from our previous path (in global coordinates)
          for (int i=0;i<prev_size;i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
            }

        

	for (int i=0;i<(50-prev_size);i++){
		next_x_vals.push_back(trajectory.x_vals[i]);
		next_y_vals.push_back(trajectory.y_vals[i]);
                }
	
//cout<<trajectory.pts_s.size()<<endl;

	//for (int i=0;i<trajectory.pts_s.size()-1;i++){
	
	//cout<<trajectory.pts_s[i]<<endl;
	
//	}
        //prev_trajectory.=next_trajectory.;

         /*
         vector<double> s=JMT(start_s,end_s,T);
         vector<double> d=JMT(start_d,end_d,T);


         vector<double> next_x_vals;
         vector<double> next_y_vals;
	 vector<double> pts_s;
         vector<double> pts_d;
	 vector<double> pts_s_dot;
         vector<double> pts_d_dot;
	 vector<double> pts_s_dot_dot;
         vector<double> pts_d_dot_dot;
         vector<double> pts_h_perp;
	 

	 for (double t=0.0;t<=T;t+=0.02){      
            pts_s.push_back(s[0]+s[1]*t+s[2]*pow(t,2)+s[3]*pow(t,3)+s[4]*pow(t,4)+s[5]*pow(t,5));
            pts_d.push_back(d[0]+d[1]*t+d[2]*pow(t,2)+d[3]*pow(t,3)+d[4]*pow(t,4)+d[5]*pow(t,5)); 
            pts_h_perp.push_back(sh_p(pts_s[pts_s.size()-1]));
            pts_s_dot.push_back(s[1]+2*s[2]*t+3*s[3]*pow(t,2)+4*s[4]*pow(t,3)+5*s[5]*pow(t,4));
	    pts_s_dot_dot.push_back(2*s[2]+6*s[3]*t+12*s[4]*pow(t,2)+20*s[5]*pow(t,3));
            pts_d_dot.push_back(d[1]+2*d[2]*t+3*d[3]*pow(t,2)+4*d[4]*pow(t,3)+5*d[5]*pow(t,4));
	    pts_d_dot_dot.push_back(2*d[2]+6*d[3]*t+12*d[4]*pow(t,2)+20*d[5]*pow(t,3));
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
        */
  
     
              
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
