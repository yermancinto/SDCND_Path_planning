#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Geometry"
#include "cppfsm-master/fsm.h"   //FSM library
#include "helpers.h"
#include "json.hpp"
#include <typeinfo>
#include "spline.h"
#include "constants.h"
#include <math.h>
#include <algorithm> //min



#include "trajectory_planner.cpp"
#include "cost_functions.cpp"

using namespace std;


// for convenience
using nlohmann::json;
using std::string;
using std::vector;

//typedef std::chrono::high_resolution_clock Time;
//typedef std::chrono::seconds s;

//Finite State Machine:
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


int main() {
  	uWS::Hub h;
	double lane=1;	//Our vehicle starts in the middle lane
	//double target_lane;
	Trajectory ego; //create first instance of type Trajectory
	bool vehicle_ahead;	
	bool vehicle_left;
	bool vehicle_right;
	int action;
	


	//Define transitions:
  	std::vector<F::Trans> transitions = 
		{
		//From State, 	To State,  	Trigger,            		Guard,                   				action
  		{States::KL,    States::KL,     Triggers::Clear,          	[&]{return true;},					[&]{action=1;}		},
  		{States::KL,    States::L_LCH,  Triggers::Vehicle_ahead,  	[&]{return (not(vehicle_left)&&(lane-1>=0));},		[&]{action=3;lane=lane-1;}       },
  		{States::KL,    States::R_LCH,  Triggers::Vehicle_ahead,  	[&]{return (not(vehicle_right)&&(lane+1<3));},		[&]{action=3;lane=lane+1;}       },
  		{States::KL,    States::FV,    	Triggers::Vehicle_ahead,  	[&]{return ( ((lane=1)&&(vehicle_left)&&(vehicle_right)) || ((lane=0)&&(vehicle_right)) || ((lane=2)&&(vehicle_left)) ) ;},													[&]{action=2;}		},

  		{States::L_LCH, States::KL,    	Triggers::Clear,  		[&]{return true;},					[&]{action=1;}   	},
  		{States::L_LCH, States::FV,     Triggers::Vehicle_ahead,  	[&]{return true;},					[&]{action=2;} 	},

  		{States::R_LCH, States::KL,     Triggers::Clear,  		[&]{return true;},					[&]{action=1;}   	},
  		{States::R_LCH, States::FV,     Triggers::Vehicle_ahead,  	[&]{return true;},					[&]{action=2;} 	},

  		{States::FV,    States::L_LCH,  Triggers::Vehicle_ahead,  	[&]{return (not(vehicle_left)&&(lane-1>=0));},      	[&]{action=3;lane=lane-1;}       },
  		{States::FV,    States::R_LCH,  Triggers::Vehicle_ahead,  	[&]{return (not(vehicle_right)&&(lane+1<3));},  	[&]{action=3;lane=lane+1;}       },
  		{States::FV,    States::KL,  	Triggers::Clear,  		[&]{return true;},					[&]{action=1;}  	},
  		{States::FV,    States::FV,  	Triggers::Vehicle_ahead,  	[&]{return(((lane=1)&&(vehicle_left)&&(vehicle_right)) || ((lane=0)&&(vehicle_right)) || ((lane=2)&&(vehicle_left)) );},														[&]{action=2;} 	},
  		};

   
//cout<<"problem 1"<<endl;

	fsm.add_transitions(transitions);
	fsm.add_debug_fn(dbg_fsm);

	h.onMessage([&ego,&fsm,&lane,&vehicle_left,&vehicle_right,&action]
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
          			double car_x = j[1]["x"];//in [m]
          			double car_y = j[1]["y"];//in [m]
          			double car_s = j[1]["s"];//in [m]
          			double car_d = j[1]["d"];//in [m]
          			double car_yaw = j[1]["yaw"];
          			double car_speed=j[1]["speed"];//in [mph]
				car_speed *= 0.44704;//in [m/s]
          			// Previous path data given to the Planner
          			auto previous_path_x = j[1]["previous_path_x"];
          			auto previous_path_y = j[1]["previous_path_y"];
          			// Previous path's end s and d values 
          			double end_path_s = j[1]["end_path_s"];
          			double end_path_d = j[1]["end_path_d"];
          			// Sensor Fusion Data, a list of all other cars on the same side 
          			//   of the road.
          			auto sensor_fusion = j[1]["sensor_fusion"];
          			int prev_size=previous_path_x.size(); //previous path size
        
				ego.points_added_to_actual_path(prev_size);




				
         			//************************************
         			//Check the environment of our vehicle
         			//************************************
				
				int ego_lane=-1;		//contains the ID of the vehicle we have ahead
				vector <int> left_lane;		//contains the ID of the vehicles to be tracked when pretending lane change left (to be used later in the cost function)
				vector <int> right_lane;	//contains the ID of the vehicles to be tracked when pretending lane change right (to be used later in the cost function)

				//************************************
				//1.vehicle ahead??
				//************************************

				//float target_vehicle_ID=0;

			
				double ref_distance=100;
				double target_vehicle_dist;
				for (int i=0; i<sensor_fusion.size();i++){ //loop over all the vehicles
       					float check_car_d=sensor_fusion[i][6]; //d position of the vehicle being tracked
            				if (check_car_d>(2+4*lane-2)&& check_car_d<(2+4*lane+2)){ //if another vehicle in our lane
              					double check_car_s=sensor_fusion[i][5];//s position of the vehicle being tracked
              					double distance=check_car_s-car_s; //distance to the vehicle being tracked
              					if ((distance < detec_thr)&&(distance>0)){ //vehicle tracked is ahead and at less than 50 meters
              						if (distance<ref_distance){ //in case there is more than 1 vehicle ahead and inside the threshold 
											//detection range, we should only track the closest one
								ego_lane=sensor_fusion[i][0];//vehicle id
                						target_vehicle_dist=distance;                
                						//double vx=sensor_fusion[i][3]; // in m/s
                						//double vy=sensor_fusion[i][4]; // in m/s
                						//target_vehicle_spd=sqrt(vx*vx+vy*vy); //speed of the vehicle ahead   	
								}                 
                					}
            					}
					}  	

				//************************************
				//2.left /right lanes free??
				//************************************
				if(ego_lane!=-1){//Look for possible gaps left and right just in case there is a vehicle in front of us
					float reference=sensor_fusion[ego_lane][5];//s value of the vehicle we are following
					if (lane!=0){//if we are in lane 1 or 2, search for space in left lane:
						for (int i=0; i<sensor_fusion.size();i++){ //loop over all the vehicles
       							float check_car_d=sensor_fusion[i][6]; //d position of the vehicle being tracked
							float check_car_s=sensor_fusion[i][5]; //s position of the vehicle being tracked
							
							if ((check_car_d<2+4*(lane-1)+2) && (check_car_d>2+4*(lane-1)-2)&& (check_car_s< reference+20) && ((car_s-20)<check_car_s)  )  {
								left_lane.push_back(sensor_fusion[i][0]); //ID of the vehicle inside the defined range
								
								
								}
							}
						}


					if (lane!=2){//if we are in lane 0 or 1, search for space in right lane:
						for (int i=0; i<sensor_fusion.size();i++){ //loop over all the vehicles
       							float check_car_d=sensor_fusion[i][6]; //d position of the vehicle being tracked
							float check_car_s=sensor_fusion[i][5]; //s position of the vehicle being tracked
	
							if ((check_car_d<2+4*(lane+1)+2) && (check_car_d>2+4*(lane+1)-2) &&(check_car_s< reference+20) && ((car_s-20)<check_car_s) ){
								right_lane.push_back(sensor_fusion[i][0]); //ID of the vehicle inside the defined range
								
								}
							}
						}
					}

				if (right_lane.size()==0){
					vehicle_right=false;
					}
				else{
					vehicle_right=true;
					}

				if (left_lane.size()==0){
					vehicle_left=false;
					}
				else{
					vehicle_left=true;
					}
				//cout<<"vehicle_left: "<<vehicle_left<<endl;
				//cout<<"vehicle_right: "<<vehicle_right<<endl;
				//cout<<"vehicle_ahead: "<<ego_lane<<endl;
				/*
				cout<<"vehcile ahead: "<<vehicle_ahead<<"at "<<target_vehicle_dist<<" m"<<endl;
				if(left_lane.size()>0){
					for (int i=0;i<left_lane.size();i++){
						cout<<"vehicle left side: "<<left_lane[i]<<endl;
						}
					}

				else{
					cout<<"left side free"<<endl;
					} 
				if(right_lane.size()>0){
					for (int i=0;i<right_lane.size();i++){
						cout<<"vehicle right side: "<<right_lane[i]<<endl;
						}
					}
				else{
					cout<<"right side free"<<endl;
					} 
				*/
				//************************************
				//FSM:
				//************************************
				
				if (ego_lane==-1){ //No vehicle ahead...
					fsm.execute(Triggers::Clear);
					}
				else {
					fsm.execute(Triggers::Vehicle_ahead);
					}	

				//************************************
				double final_s;
				double final_s_dot;
				end_path_s=fmod(end_path_s,6945.554);
				//vehicle state at the end of the actual running path
				if(prev_size>0){
					car_speed=ego.start_s[1];
					double car_acceleration=ego.start_s[2];
					}
				
				
				cout<<"action: "<<action<<endl;

				switch(action){
					case 1: 
						{//Keep lane case:

						if(car_speed<speed_limit){//ego vehicle actually driving below speed limit
							
							if ((car_speed+max_accel*time_to_target)<speed_limit){
								final_s=end_path_s+car_speed*time_to_target+0.5*max_accel*pow(time_to_target,2);
								final_s_dot=car_speed+max_accel*time_to_target;
								}
							else{
								final_s=end_path_s+car_speed*time_to_target+0.5*max_accel*pow(time_to_target,2);
								final_s_dot=speed_limit;
								}
							}
						else{//ego vehicle actually driving at speed limit
							final_s=end_path_s+speed_limit*time_to_target;
							final_s_dot=speed_limit;
							}
						break;
						}

	
					case 2:
						{//Follow vehicle case:
						//Trajectory vehicle_in_front;//create new instance for the vehicle in front of us
						Trajectory vehicle_in_front;

						auto [s_dot_front,d_dot_front]=vehicle_in_front.speed_to_frenet(sensor_fusion[ego_lane][5],sensor_fusion[ego_lane][6],sensor_fusion[ego_lane][3],sensor_fusion[ego_lane][4]);
cout<<"vehicle ahead at "<<s_dot_front<<"m/s"<<endl;
						vector<double> actual_s={sensor_fusion[ego_lane][5],s_dot_front,0};//actual s state of the vehicle in front of us
						vector<double> actual_d={sensor_fusion[ego_lane][6],d_dot_front,0};//actual d state of the vehicle in front of us
						
						auto predicted_state =vehicle_in_front.predict_state(actual_s,actual_d,time_to_target);
						final_s=predicted_state[0][0]-distance_FV;
						final_s_dot=s_dot_front;
						cout<<"ego speed: "<<car_speed<<"m/s"<<endl;
						
						break;
						}
						//Trajectory::~vehicle_in_front;

					case 3:
						{//Lane change case (left or right):
						//Trajectory vehicle_in_front;//create new instance for the vehicle in front of us
						time_to_target=3;
						Trajectory vehicle_in_front;

						auto [s_dot_front,d_dot_front]=vehicle_in_front.speed_to_frenet(sensor_fusion[ego_lane][5],sensor_fusion[ego_lane][6],sensor_fusion[ego_lane][3],sensor_fusion[ego_lane][4]);
						vector<double> actual_s ={sensor_fusion[ego_lane][5],s_dot_front,0};//actual s state of the vehicle in front of us
						vector<double> actual_d={sensor_fusion[ego_lane][6],d_dot_front,0};//actual d state of the vehicle in front of us
						auto predicted_state=vehicle_in_front.predict_state(actual_s,actual_d,time_to_target);


						//if(car_speed<speed_limit){//ego vehicle actually driving below speed limit
						if(car_speed<speed_limit){//ego vehicle actually driving below speed limit
							
							if ((car_speed+max_accel*time_to_target)<speed_limit){
								final_s=end_path_s+car_speed*time_to_target+0.5*max_accel*pow(time_to_target,2);
								final_s_dot=car_speed+max_accel*time_to_target;
								}
							else{
								final_s=end_path_s+car_speed*time_to_target+0.5*max_accel*pow(time_to_target,2);
								final_s_dot=speed_limit;
								}
							}
						else{//ego vehicle actually driving at speed limit
							final_s=end_path_s+speed_limit*time_to_target;
							final_s_dot=speed_limit;
							}

						break;
						}}




          			if (prev_size>0){
					vector<double> f_s={final_s,final_s_dot,0.0};				
     					vector<double> f_d={2+4*lane,0.0,0.0};
	        			ego.set_final_state(f_s,f_d);
 					ego.get_trajectory(time_to_target,0.02);
					ego.set_next_start();
					}

				else{ //there is no previous path, so start a new path from actual vehicle state:
					double time_to_target_1=5;
                 			final_s=car_s+0.5*max_accel*pow(time_to_target_1,2);
					final_s_dot=max_accel*time_to_target_1;
					vector<double> i_s={car_s,car_speed,0.0};	//initial s state 
		 			vector<double> i_d={car_d,0.0,0.0};		//initial d state 
                 			vector<double> f_s={final_s,final_s_dot,0.0};//final s state
                 			vector<double> f_d={2+4*lane,0.0,0.0};		//final d state
 					ego.set_initial_state(i_s,i_d);
	        			ego.set_final_state(f_s,f_d);
		 			ego.get_trajectory(time_to_target_1,0.02);
					ego.set_next_start();
         				}
/*
        			double final_lane=2+4*lane;
				Trajectory best;
				if (prev_size>0){
//vector<double>initial_s,vector<double>initial_d, double nominal_s_goal, double nominal_d_goal,double final_s_dot,const double sigma_s,const double sigma_d,const int n_samples,double T,double time_step)
					vector<Trajectory> trajectories_1=trajectories(ego.start_s,ego.start_d,final_s,final_lane,final_s_dot,sigma_s,sigma_d,n_samples,time_to_target,0.1);
					vector<double> trajectory_cost;
					for (int i=0;i<trajectories_1.size();i++){
 						auto cost=calculate_cost(trajectories_1[i]);
						trajectory_cost.push_back(cost);
						}
					int min_index=min_element(trajectory_cost.begin(),trajectory_cost.end())-trajectory_cost.begin();
					best=trajectories_1[min_index];
					double lower_cost=calculate_cost(trajectories_1[min_index]);
					cout<<"lower_cost: "<<lower_cost<<endl;
					}



         			if (prev_size>0){
					//vector<double> f_s={final_s,final_s_dot,0.0};				
					vector<double> f_s=best.end_s;     					
					//vector<double> f_d={2+4*lane,0.0,0.0};
					vector<double> f_d=best.end_d;
	        			ego.set_final_state(f_s,f_d);
					ego.get_trajectory(best.target_time,0.02);
					ego.set_next_start();
					}

				else{ //there is no previous path, so start a new path from actual vehicle state:
					double time_to_target_1=5;
                 			final_s=car_s+0.5*max_accel*pow(time_to_target_1,2);
					final_s_dot=max_accel*time_to_target_1;
					vector<double> i_s={car_s,car_speed,0.0};	//initial s state 
		 			vector<double> i_d={car_d,0.0,0.0};		//initial d state 
                 			vector<double> f_s={final_s,final_s_dot,0.0};//final s state
                 			vector<double> f_d={2+4*lane,0.0,0.0};		//final d state
 					ego.set_initial_state(i_s,i_d);
	        			ego.set_final_state(f_s,f_d);
		 			ego.get_trajectory(time_to_target_1,0.02);
					ego.set_next_start();
         				}

*/

         			vector<double> next_x_vals;
          			vector<double> next_y_vals;


				//our trajectory is always formed by 50 points:
          			//start with all the points from our previous path (in global coordinates)
          			for (int i=0;i<prev_size;i++){
            				next_x_vals.push_back(previous_path_x[i]);
            				next_y_vals.push_back(previous_path_y[i]);
 					}
        
				//cout<<50-prev_size<<"values added to previous path"<<endl;

				int next_size=ego.x_vals.size();
				for (int i=0;i<next_size;i++){
					next_x_vals.push_back(ego.x_vals[i]);
					next_y_vals.push_back(ego.y_vals[i]);
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
