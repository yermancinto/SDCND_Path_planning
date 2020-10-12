
#include <iostream>
#include <fstream>
#include <math.h>
#include <string>
#include <vector>
#include "spline.h"
#include "trajectory_planner.h"
#include <tuple>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// for convenience
using std::string;
using std::vector;
using namespace std;


void Trajectory::JMT_s(double T) {
	MatrixXd A = MatrixXd(3, 3);
  	A << T*T*T, T*T*T*T, T*T*T*T*T,
       	3*T*T, 4*T*T*T,5*T*T*T*T,
       	6*T, 12*T*T, 20*T*T*T;
  	
	MatrixXd B = MatrixXd(3,1);     
  	B << end_s[0]-(start_s[0]+start_s[1]*T+.5*start_s[2]*T*T),
       	end_s[1]-(start_s[1]+start_s[2]*T),
       	end_s[2]-start_s[2];

  	MatrixXd Ai = A.inverse();
  	MatrixXd C = Ai*B;
	s.clear();
  	s = {start_s[0], start_s[1], .5*start_s[2]};
  	for(int i = 0; i < C.size(); ++i) {
    		s.push_back(C.data()[i]);
    		}
	//cout<<"s coefficients:"<<s[0]<<";"<<s[1]<<";"<<s[2]<<";"<<s[3]<<";"<<s[4]<<";"<<s[5]<<endl;
	
	}


void Trajectory::JMT_d(double T) {
	MatrixXd A = MatrixXd(3, 3);
  	A << T*T*T, T*T*T*T, T*T*T*T*T,
       	3*T*T, 4*T*T*T,5*T*T*T*T,
       	6*T, 12*T*T, 20*T*T*T;
  	
	MatrixXd B = MatrixXd(3,1);     
  	B << end_d[0]-(start_d[0]+start_d[1]*T+.5*start_d[2]*T*T),
       	end_d[1]-(start_d[1]+start_d[2]*T),
       	end_d[2]-start_d[2];

  	MatrixXd Ai = A.inverse();
  	MatrixXd C = Ai*B;
	d.clear();
  	d = {start_d[0], start_d[1], .5*start_d[2]};
  	for(int i = 0; i < C.size(); ++i) {
    		d.push_back(C.data()[i]);
    		}
	}

void Trajectory::calculate_frenet_points(double T,double deltaT){
	 pts_s.clear();
         pts_d.clear();
         pts_h_perp.clear();
	 pts_s_dot.clear();
         pts_s_dot_dot.clear();
	 pts_s_dot_dot_dot.clear();
         pts_d_dot.clear();
         pts_d_dot_dot.clear();
	 pts_d_dot_dot_dot.clear();


	
	 //for (double t=0.0;t<=T;t+=0.02){    
	for (double t=0.0;t<=T;t+=deltaT){   
            pts_s.push_back(fmod(s[0]+s[1]*t+s[2]*pow(t,2)+s[3]*pow(t,3)+s[4]*pow(t,4)+s[5]*pow(t,5),max_s)); //with the modulo "fmod", we keep the S value inside the range [0,max_s]->usefull when crossing the finish line of our map


            pts_d.push_back(d[0]+d[1]*t+d[2]*pow(t,2)+d[3]*pow(t,3)+d[4]*pow(t,4)+d[5]*pow(t,5)); 
            pts_h_perp.push_back(sh_p_map(pts_s[pts_s.size()-1]));
		
	    //cout<<"s:"<<s[0]+s[1]*t+s[2]*pow(t,2)+s[3]*pow(t,3)+s[4]*pow(t,4)+s[5]*pow(t,5)<<endl;
            //cout<<"d:"<<d[0]+d[1]*t+d[2]*pow(t,2)+d[3]*pow(t,3)+d[4]*pow(t,4)+d[5]*pow(t,5)<<endl;
	    //speeds:
            pts_s_dot.push_back(s[1]+2*s[2]*t+3*s[3]*pow(t,2)+4*s[4]*pow(t,3)+5*s[5]*pow(t,4));
		pts_d_dot.push_back(d[1]+2*d[2]*t+3*d[3]*pow(t,2)+4*d[4]*pow(t,3)+5*d[5]*pow(t,4));
	    
            //accelerations:
            pts_s_dot_dot.push_back(2*s[2]+6*s[3]*t+12*s[4]*pow(t,2)+20*s[5]*pow(t,3));
	    pts_d_dot_dot.push_back(2*d[2]+6*d[3]*t+12*d[4]*pow(t,2)+20*d[5]*pow(t,3));

	//jerks:
		pts_s_dot_dot_dot.push_back(6*s[3]+24*s[4]*t+60*s[5]*pow(t,2));
		pts_d_dot_dot_dot.push_back(6*d[3]+24*d[4]*t+60*d[5]*pow(t,2));
	    }

	
	}



void Trajectory::calculate_xy_points(){
         double size=pts_s.size();
         x_vals.clear(); //clear previous values
         y_vals.clear();

	 for (int i=1;i<points_to_add+1;i++){
		
		double x= sx_map(pts_s[i]);
		double y= sy_map(pts_s[i]);
			
	        x+=pts_d[i]*cos(pts_h_perp[i]);	//in [m]
		y+=pts_d[i]*sin(pts_h_perp[i]); //in [m]
		//cout<<"x:"<<x<<endl;
                //cout<<"y:"<<y<<endl;
	        x_vals.push_back(x);
        	y_vals.push_back(y);
		}

          }

vector<double> Trajectory::derivatives(double s_value){

	
	for (int i=0;i<pts_s.size();i++){
           if (pts_s[i]>=s_value){
		//cout<<"pts_s:"<<pts_s[i]<<endl;                
		return{pts_s_dot[i],pts_s_dot_dot[i],pts_d_dot[i],pts_d_dot_dot[i]};
		}
            else{
                 //cout<<"not found"<<endl;
                 }	
         }


}



void Trajectory::get_trajectory(double T,double deltaT){

	JMT_s(T);
	JMT_d(T);
	calculate_frenet_points(T,deltaT);
	calculate_xy_points();
	//start_s={fmod(pts_s[points_to_add],max_s),pts_s_dot[points_to_add],pts_s_dot_dot[points_to_add]};
	//start_d={pts_d[points_to_add],pts_d_dot[points_to_add],pts_d_dot_dot[points_to_add]};

	}

void Trajectory::set_initial_state(vector<double> initial_s,vector<double> initial_d){
	start_s=initial_s;
	start_d=initial_d;
	}

void Trajectory::set_final_state(vector<double> final_s,vector<double> final_d){
	end_s=final_s;
	end_d=final_d;
	}

void Trajectory::use_prev_path(){
	start_s={pts_s[points_to_add],pts_s_dot[points_to_add],pts_s_dot_dot[points_to_add]};
	start_d={pts_d[points_to_add],pts_d_dot[points_to_add],pts_d_dot_dot[points_to_add]};
	}

void Trajectory::points_added_to_actual_path(int points){
	points_to_add=trajectory_points-points;
	}

void Trajectory::set_next_start(){
	start_s={fmod(pts_s[points_to_add],max_s),pts_s_dot[points_to_add],pts_s_dot_dot[points_to_add]};
	start_d={pts_d[points_to_add],pts_d_dot[points_to_add],pts_d_dot_dot[points_to_add]};
	}

void Trajectory::set_time_to_target(double time){
	target_time=time;
	}
//********************
//Auxiliary functions:
//********************

//Predicts the final state given the initial state (s&d) and the time T:
vector<vector<double>> Trajectory::predict_state(vector<double>actual_s,vector<double>actual_d,double T){
	vector<double> final_s;
	vector<double> final_d;
	final_s={actual_s[0]+actual_s[1]*T,actual_s[1],0};
	final_d={actual_d[0]+actual_d[1]*T,actual_d[1],actual_d[2],0};
	return {final_s,final_d};
	}
//transforms the speed of the target vehicle from cartesian to Frenet coordinates:
tuple<double,double> Trajectory::speed_to_frenet(double s,double d,double vx,double vy){
	double speed=sqrt(pow(vx,2)+pow(vy,2));
	cout<<"vehicle ahead speed: "<<speed<<"m/s"<<endl;
	double theta=-sh_p_map(s);//in [rad]
	cout<<"angle: "<<theta<<endl;
	cout<<"s: "<<s<<endl;
	double s_dot=vx*sin(theta)+vy*cos(theta); //in [m/s]
	double d_dot=vx*cos(theta)-vy*sin(theta); //in [m/s]
	cout<<"vehicle ahead speed_s_dot "<<s_dot<<"m/s"<<endl;
	cout<<"vehicle ahead speed_d_dot "<<d_dot<<"m/s"<<endl;
	return {s_dot,d_dot}; //
	}
//predicts the trajectory of a vehicle
vector<vector<double>> Trajectory::predict_trajectory(vector<double>actual_s,vector<double>actual_d,vector<double>final_s,vector<double>final_d,double T,double t){
	set_initial_state(actual_s,actual_d);
	set_final_state(final_s,final_d);
	get_trajectory(T,t);
	return{x_vals,y_vals};
	}
	
	

	








