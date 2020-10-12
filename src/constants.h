
const int n_samples=5;
const double sigma_s=2;//[m]
const double sigma_d=0.5;//[m]
const double sigma_s_dot=5;//[m/s]
const double mph_speed_limit=50.0;//[mph]-> to meters per second
const double max_jerk=10.0;//[m/s³]
const double max_accel=2.0;//(total vehicle acceleration)[m/s²] 
const double expected_jerk_in_one_sec=2.0;
const double expected_acceleration_in_one_sec=1.0;
const double vehicle_radius=1.5; //[m]

const int trajectory_points=50;

double time_to_target=5.0; //[s]

//Use international units in our code:
const double speed_limit= mph_speed_limit * 0.44704*0.85;//[m/s]

//Keep plane:
const double path_extension=110.0; //[m]

//Follow vehicle setup:
const double detec_thr=50.0;//[m]Forward sensor detectetion threshold
const double distance_FV=20.0; //[m]
