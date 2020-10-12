

double exceeds_speed_limit_cost(vector<double> pts_s_dot,vector<double> pts_d_dot);
double exceed_acceleration_cost(vector<double>pts_s_dot_dot,vector<double>pts_d_dot_dot);
double collision_cost(Trajectory trajectory_ego,Trajectory trajectory_target);
double calculate_cost(Trajectory trajectory_ego,Trajectory trajectory_target);

vector <Trajectory> trajectories(vector<double>initial_s,vector<double>initial_d, double nominal_s_goal, double nominal_d_goal,double final_s_dot,const double sigma_s,const double sigma_d,const int n_samples,double T,double time_step);

vector<double> perturbed_goal(double goal_s,double goal_d,double sigma_s,double sigma_d);
double nearest_approach_to_vehicle(Trajectory trajectory_ego,Trajectory trajectory_target);

