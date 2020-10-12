
#include <iostream>
#include <fstream>
#include <math.h>
#include <string>
#include <vector>
#include "spline.h"


vector<tk::spline> load_map(){

	// Waypoint map to read from
	const string map_file_ = "../data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;
	tk::spline sx_map_0 ;
	tk::spline sy_map_0;
	tk::spline sh_p_map_0 ;

	// Waypoint map to read from
	//const string map_file_ = "../data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	//double max_s = 6945.554;
	ifstream in_map_(map_file_.c_str(), ifstream::in);
	string line;

	while (getline(in_map_, line)) {
		istringstream iss(line);
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
	map_waypoints_x.push_back(map_waypoints_x[0]);
	map_waypoints_y.push_back(map_waypoints_y[0]);
	map_waypoints_s.push_back(max_s);
	map_waypoints_dx.push_back(map_waypoints_dx[0]);
	map_waypoints_dy.push_back(map_waypoints_dy[0]);

	vector<double> map_waypoints_perp_h;
	int size=map_waypoints_x.size();
       

	for (int i=0;i<size;i++){
		double value;
		value= atan2(map_waypoints_dy[i],map_waypoints_dx[i]);// [radians]
		if (map_waypoints_perp_h.size()>0){
			if ((M_PI/2<map_waypoints_perp_h[i-1])&&(map_waypoints_perp_h[i-1]<M_PI)&&(value<-M_PI/2)){
				value=value+2*M_PI;
				}
			if ((map_waypoints_perp_h[i-1]>M_PI)&&(value<0)){
				value=value+2*M_PI;
				}
			}
		map_waypoints_perp_h.push_back(value);
		//cout<<value<<endl;
		}

	//Splines generation:
	sx_map_0.set_points(map_waypoints_s,map_waypoints_x);//position x=>x(s)
	sy_map_0.set_points(map_waypoints_s,map_waypoints_y);//position y=>y(s) 
	//sh_map.set_points(map_waypoints_s,map_waypoints_h);//heading=>heading(s) 
	sh_p_map_0.set_points(map_waypoints_s,map_waypoints_perp_h);//perpendicular heading=>perpendicular heading(s) 
	cout<<"map loaded"<<endl;
	return {sx_map_0,sy_map_0,sh_p_map_0};
	
	}
