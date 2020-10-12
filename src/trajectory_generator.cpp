
#include <iostream>
#include <fstream>
#include <math.h>
#include <string>
#include <vector>
#include "spline.h"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// for convenience
using std::string;
using std::vector;
using namespace std;

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

vector<double> map_waypoints_h;
vector<double> map_waypoints_perp_h;
int size=map_waypoints_x.size();

/*for (int i=0;i<size;i++){
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
*/


for (int i=0;i<size;i++){
	double value;
	value= atan2(map_waypoints_dy[i]/map_waypoints_dx[i]);//in radians	
	map_waypoints_perp_h.push_back(value);
}

//Splines generation:
sx.set_points(map_waypoints_s,map_waypoints_x);//position x=>x(s)
sy.set_points(map_waypoints_s,map_waypoints_y);//position y=>y(s) 
sh.set_points(map_waypoints_s,map_waypoints_h);//heading=>heading(s) 
sh_p.set_points(map_waypoints_s,map_waypoints_perp_h);//perpendicular heading=>perpendicular heading(s) 



class Trajectory{


	public:
		//Load the splines in the class:
		const tk::spline sx_map = sx;
		const tk::spline sy_map = sy;
		const tk::spline sh_map = sh;
		const tk::spline sh_p_map = sh_p;

		const T=0.02; //time step defined by the simulator[s]

		//define states:
 		vector <double> start_s;    	
		vector <double> end_s;
 		vector <double> start_d;    	
		vector <double> end_d;

                vector <double> x_vals;
                vector <double> y_vals;

           
	


                s=JMT(start_s,end_s,T);//s quintic polynomial coeficients
		d=JMT(start_d,end_d,T);//d quintic polynomial coeficients

                vector<double> pts_s;
         	vector<double> pts_d;
		vector<double> pts_s_dot;
		vector<double> pts_d_dot;
		vector<double> pts_s_dot_dot;
		vector<double> pts_d_dot_dot;
		vector<double> pts_h_perp;


};

//Jerk Minimizing trajectory:
vector<double> Trajectory::JMT(vector<double> &start, vector<double> &end, double T) {
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

  	vector <double> coeficients = {start[0], start[1], .5*start[2]};
  	for(int i = 0; i < C.size(); ++i) {
    		coeficients.push_back(C.data()[i]);
    		}
  	return coeficients;
	}

void Trajectory::calculate_frenet_points(){
	 for (double t=0.0;t<=T;t+=0.02){      
            pts_s.push_back(s[0]+s[1]*t+s[2]*pow(t,2)+s[3]*pow(t,3)+s[4]*pow(t,4)+s[5]*pow(t,5));
            pts_d.push_back(d[0]+d[1]*t+d[2]*pow(t,2)+d[3]*pow(t,3)+d[4]*pow(t,4)+d[5]*pow(t,5)); 
            pts_h_perp.push_back(sh_p(pts_s[pts_s.size()-1]));

            pts_s_dot.push_back(s[1]+2*s[2]*t+3*s[3]*pow(t,2)+4*s[4]*pow(t,3)+5*s[5]*pow(t,4));
	    pts_s_dot_dot.push_back(2*s[2]+6*s[3]*t+12*s[4]*pow(t,2)+20*s[5]*pow(t,3));
            pts_d_dot.push_back(d[1]+2*d[2]*t+3*d[3]*pow(t,2)+4*d[4]*pow(t,3)+5*d[5]*pow(t,4));
	    pts_d_dot_dot.push_back(2*d[2]+6*d[3]*t+12*d[4]*pow(t,2)+20*d[5]*pow(t,3));
	    }
	}

void Trajectory::calculate_xy_points(){
         double size=pts_s.size();
	 for (int i=0;i<=size;i++){
		double x= sx(pts_s[i]);
		double y= sy(pts_s[i]);
	        x=x+pts_d[i]*cos(pts_h_perp[i]);	
		y=y+pts_d[i]*sin(pts_h_perp[i]);
	        x_vals.push_back(x);
        	y_vals.push_back(y);
		}
          }

vector<double> Trajectory::derivatives(double s){
	for (i=0;i<pts_s.size();i++){
		if (pts_s[i]>=s){
			return {pts_s_dot[i],pts_s_dot_dot[i],pts_d_dot[i],pts_d_dot_dot[i]};
			}
		else{
			std::cout<<"s outside the range of the previous trajectory"<<enl;
		}
	}




