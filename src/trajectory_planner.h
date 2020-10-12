#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <string>
#include <vector>
#include "spline.h"
#include <algorithm>
#include <typeinfo>
#include "map.cpp"
#include <tuple>

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// for convenience
using std::string;
using std::vector;
using namespace std;




class Trajectory{
             

		public:
	        //Trajectory(); 	//Constructor
                //~Trajectory();	//Destructor
	
/*
	// Waypoint map to read from
	const string map_file_ = "../data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;
		


		//Load the splines in the class:
		tk::spline sx_map ;
		tk::spline sy_map ;
		tk::spline sh_map ;
		tk::spline sh_p_map ;

		double T=time_to_target;
		int points_to_add;

*/
		tk::spline sx_map=load_map()[0];
		tk::spline sy_map=load_map()[1];
		tk::spline sh_p_map=load_map()[2];

		//sx_map=load_map()[0];
		double max_s = 6945.554;
		

		//double T=time_to_target;
		int points_to_add;
		//define states:
 		vector <double> start_s; //start s state   	
		vector <double> end_s;   //end s state
 		vector <double> start_d; //start d state   	
		vector <double> end_d;	 //end d state
		double target_time;	//time to target

                vector <double> x_vals; //output x points
                vector <double> y_vals; //output y points
		

           	
 		vector <double> s;//s quintic polynomial coeficients
		vector <double> d;//d quintic polynomial coeficients

                vector<double> pts_s; //output s values
         	vector<double> pts_d; //output d values
		vector<double> pts_s_dot; //output s velocity points
		vector<double> pts_d_dot; //output d velocity points
		vector<double> pts_s_dot_dot;//output s acceleration points
		vector<double> pts_d_dot_dot;//output d acceleration points
		vector<double> pts_s_dot_dot_dot;//output s jerk points
		vector<double> pts_d_dot_dot_dot;//output d jerk points

		vector<double> pts_h_perp; //in [rad]
		
		//void load_map();

		void set_initial_state(vector <double> initial_s,vector <double> initial_d);
		void set_final_state(vector <double> final_s,vector <double> final_d);
		void set_time_to_target(double time);

		void JMT_s(double T);
		void JMT_d(double T);

		void calculate_frenet_points(double T ,double deltaT);
		void use_prev_path();

		void calculate_xy_points();
		vector<double> derivatives(double s_value);
		void get_trajectory(double T,double deltaT);

		void points_added_to_actual_path(int points);
		void set_next_start();

		vector<vector<double>> predict_state(vector<double>actual_s,vector<double>actual_d,double T);
		tuple<double,double> speed_to_frenet(double s,double d,double vx,double vy);
		vector<vector<double>> predict_trajectory(vector<double>actual_s,vector<double>actual_d,vector<double>final_s,vector<double>final_d,double T,double t);
		};

#endif

