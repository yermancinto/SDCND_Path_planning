#include "cost_functions.h"
#include <math.h>	

#include <random>

//****************************************************************************************
//***************************EXCEED SPEED COST (BINARY COST)******************************
//****************************************************************************************
double exceeds_speed_limit_cost(vector<double> pts_s_dot,vector<double> pts_d_dot){
	for (int i=0;i<pts_s_dot.size();++i){
		double speed= sqrt(pow(pts_s_dot[i],2)+pow(pts_d_dot[i],2));
		if (speed>speed_limit){
			return 1.0;		
			}		
		}
	return 0.0;
	}
//****************************************************************************************
//***************************EXCEED ACCELERATION COST (BINARY COST)***********************
//****************************************************************************************
double exceed_acceleration_cost(vector<double>pts_s_dot_dot,vector<double>pts_d_dot_dot){
	for (int i=0;i<pts_s_dot_dot.size();++i){
		double accel= sqrt(pow(pts_s_dot_dot[i],2)+pow(pts_d_dot_dot[i],2));
		if (accel>max_accel){
			return 1.0;		
			}		
		}
	return 0.0;
	}
//****************************************************************************************
//***************************EXCEED JERK COST (BINARY COST)*******************************
//****************************************************************************************
double exceed_jerk_cost(vector<double>pts_s_dot_dot_dot,vector<double>pts_d_dot_dot_dot){
	for (int i=0;i<pts_s_dot_dot_dot.size();++i){
		double jerk= sqrt(pow(pts_s_dot_dot_dot[i],2)+pow(pts_d_dot_dot_dot[i],2));
		if (jerk>max_jerk){
			return 1.0;		
			}		
		}
	return 0.0;
	}
//****************************************************************************************
//***************************COLLISION COST (BINARY COST)*********************************
//****************************************************************************************
double collision_cost(Trajectory trajectory_ego,Trajectory trajectory_target){
	//for (int i=0;i<target_vehicles;i++){
		double nearest=nearest_approach_to_vehicle(trajectory_ego,trajectory_target);
		if (nearest<2*vehicle_radius){
			return 1.0;
		}
	return 0.0;
	}
//****************************************************************************************
//***************************CALCULATE WHOLE COST*****************************************
//****************************************************************************************
double calculate_cost(Trajectory trajectory_ego){
	double cost;
	double speed_limit_cost=exceeds_speed_limit_cost(trajectory_ego.pts_s_dot,trajectory_ego.pts_d_dot);
	//double collision_with_target_cost=collision_cost(trajectory_ego,trajectory_target);
	//double maximum_jerk_cost=exceed_jerk_cost(trajectory_ego.pts_s_dot_dot_dot,trajectory_ego.pts_d_dot_dot_dot);
	//double maximum_acceleration_cost=exceed_acceleration_cost(trajectory_ego.pts_s_dot_dot,trajectory_ego.pts_d_dot_dot);
	cost=speed_limit_cost;//+maximum_jerk_cost+maximum_acceleration_cost;
	return cost;
	}




//****************************************************************************************
//***************************GENERATE ALL POSSIBLE TRAJECTORIES***************************
//****************************************************************************************

//vector <tuple < int,vector<double>,vector<double>,vector<double>,vector<double> >> trajectories(vector<double>initial_s,vector<double>initial_d, double nominal_s_goal, double nominal_d_goal,double final_s_dot,double sigma_s,double sigma_d,int n_samples,double T,double time_step){

vector <Trajectory> trajectories(vector<double>initial_s,vector<double>initial_d, double nominal_s_goal, double nominal_d_goal,double final_s_dot,const double sigma_s,const double sigma_d,const int n_samples,double T,double time_step){
	//time_step=0.25;//[s]
	vector <vector<double>> goals;//this vector will collect all the possible goals
cout<<"point #1"<<endl;
	//add the Nominal case:
	goals.push_back({nominal_s_goal,nominal_d_goal,T});
	//add the rest of Goals: target time and some variation + target s,d values, distorsioned by nornal distribution:
	for (double t=T-4*time_step; t<=T+4*time_step;t=t+time_step){
		for (int i=0;i<n_samples;i++){
			auto disturbed_goal=perturbed_goal(nominal_s_goal,nominal_d_goal,sigma_s,sigma_d);
			goals.push_back({disturbed_goal[0],disturbed_goal[1],t});
			cout<<"point #2"<<endl;
			}
		}
	//     		trajectory ID,	x points,	y points,	s velocity,	d velocity,	s_acceleration,	d acceleration,	s jerk, 	d jerk
	//vector <tuple < int,		vector<double>,	vector<double>,	vector<double>,	vector<double>, vector<double>,	vector<double>,	vector<double>,	vector<double>>> trajectories;
	vector<Trajectory> trajectories;
cout<<"point #3"<<endl;
	for (int i=0;i<goals.size();i++){//loop over all posible goals to create all possible trajectories
		cout<<"point #4"<<endl;
		Trajectory draft;
cout<<"point #5"<<endl;
		draft.set_initial_state(initial_s,initial_d);
		draft.set_final_state({goals[i][0],final_s_dot,0},{goals[i][1],0,0});
		draft.set_time_to_target(goals[i][2]);

cout<<"point #6"<<endl;
cout<<draft.start_d[0]<<endl;
cout<<draft.start_d[1]<<endl;
cout<<draft.start_d[2]<<endl;
//cout<<goals[i][2]<<endl;


		draft.get_trajectory(goals[i][2],0.1);
		//tuple <int,vector<double>,vector<double>,vector<double>,vector<double>,vector<double>,vector<double>> trajectory;		
		//auto trajectory = make_tuple(i,draft.x_vals,draft.y_vals,draft.pts_s_dot,draft.pts_d_dot,draft.pts_s_dot_dot,draft.pts_d_dot_dot,draft.pts_s_dot_dot_dot,draft.pts_d_dot_dot_dot);
	cout<<"point #7"<<endl;	
trajectories.push_back(draft);

		}
cout<<"point #8"<<endl;
	return trajectories;
	}



//****************************************************************************************
//***************************AUXILIARY FUNCTIONS******************************************
//****************************************************************************************
//***************************PERTURBED GOAL***********************************************
vector<double> perturbed_goal(double goal_s,double goal_d,double sigma_s,double sigma_d){
	default_random_engine generator;
	normal_distribution <double> distribution_s(goal_s,sigma_s);
	normal_distribution <double> distribution_d(goal_d,sigma_d);
	double disturbed_s= distribution_s(generator);
	double disturbed_d= distribution_d(generator);
	return {disturbed_s,disturbed_d};
	}
//***************************NEAREST APPROACH TO OTHER VEHICLE****************************
double nearest_approach_to_vehicle(Trajectory trajectory_ego,Trajectory trajectory_target){
	double min_distance=99999;	
	for (int i=0;i<trajectory_ego.x_vals.size();i++){
		double dist=abs(sqrt(pow((trajectory_ego.x_vals[i]-trajectory_target.x_vals[i]),2)+pow((trajectory_ego.y_vals[i]-trajectory_target.y_vals[i]),2)));
		if (dist<min_distance){	
			min_distance=dist;
			}	
		}
	return min_distance;
	}
//****************************************************************************************




