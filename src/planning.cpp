#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include "json.hpp"
#include "spline.h"
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
#include "planning.h"
constexpr double pi() { return M_PI; }
constexpr double deg2rad(double x) { return x * pi() / 180; }
constexpr double rad2deg(double x) { return x * 180 / pi(); }
constexpr double MPH_to_MPS(double x) { return x * 0.44074; }
#define MAX_LANE_NO 2
#define MIN_LANE_NO 0
float LANE_WIDTH = 4.0; 
float  MAX_S = 6945.554;  
float MAX_SENSOR_RANGE = 200;
float SPEED_LIMIT = MPH_to_MPS(50.0);
float ACCELERATION_LIMIT = MPH_to_MPS(10.0);

void Vehicle::updateData( float x, float y, float vx, float vy, float s, float d)
{
  this->x = x;
  this->y = y;
  this->vx = vx;
  this->vy = vy;
  this->s = s;
  this->d = d;
  lane =  int( d / LANE_WIDTH);
}
Vehicle::Vehicle(int id, float x, float y, float vx, float vy, float s, float d)
{
  this->id = id;
  updateData(x, y, vx, vy, s, d);
}
Vehicle::Vehicle()
{
  this->id = -1;
}

EgoVehicle::EgoVehicle(float x, float y, float s, float d, float yaw, float speed):
  Vehicle(-1, x, y, speed * cos(deg2rad(yaw)), speed * sin(deg2rad(yaw)), s, d)
{
  this->yaw = yaw;
  this->speed = MPH_to_MPS(speed); // convert from mph to m/s 
}
EgoVehicle::EgoVehicle():Vehicle()
{
}
void EgoVehicle::updateData(float x, float y, float s, float d, float yaw, float speed)
{
   this->yaw = yaw;
   this->speed= MPH_to_MPS(speed);
   Vehicle::updateData(x, y, speed * cos(deg2rad(yaw)), speed * sin(deg2rad(yaw)), s, d);
} 
PathPlanner::PathPlanner()
{
	target_speed = SPEED_LIMIT;
	preferred_buffer = MPH_to_MPS(0.8);
}

void PathPlanner::updateFusionSensor(vector<vector<float>> &sensor_fusion)
{
	vehicle_list.clear();
	for(int i = 0; i < sensor_fusion.size(); i++) {
	  int id = sensor_fusion[i][0];
	  float x = sensor_fusion[i][1];
	  float y = sensor_fusion[i][2];
	  float vx = sensor_fusion[i][3];
	  float vy = sensor_fusion[i][4];
	  float s = sensor_fusion[i][5];
	  float d = sensor_fusion[i][6];
	  Vehicle car(id, x, y, vx, vy, s, d);
	  car.speed = (sqrt(vx*vx + vy*vy));
	  vehicle_list.push_back(car);
	}    
}
void PathPlanner::updateEgoCar(float x, float y, float s, float d, float yaw, float speed)
{
  	ego.updateData(x, y, s, d, yaw, speed);
}
void PathPlanner::updatePath(vector<double> previous_path_x, vector<double> previous_path_y, double end_s, double end_d)
{
	this->previous_path_x = previous_path_x;
	this->previous_path_y = previous_path_y;
	this->end_d = end_d;
	this->end_s = end_s;
}
void PathPlanner::realizeState(
    vector<double> &map_waypoints_x, vector<double> &map_waypoints_y, vector<double> &map_waypoints_s,
	vector<double> &next_x_vals, vector<double> &next_y_vals)
{
	if ( prev_state != state)
	   cout << "realize state: " << state << endl;
	switch( state) {
		case STATE_LANE_CHANGE_LEFT:		   
		   if ( prev_state == STATE_PREPARE_LANE_CHANGE_LEFT)
			   target_lane -= 1;
		   generate_trajectory(target_lane,
		   	   map_waypoints_x, map_waypoints_y, map_waypoints_s,
 		   	   next_x_vals, next_y_vals);
		   break;
		case STATE_LANE_CHANGE_RIGHT:
		   if ( prev_state == STATE_PREPARE_LANE_CHANGE_RIGHT)
			   target_lane += 1;
		   generate_trajectory(target_lane,
		   	   map_waypoints_x, map_waypoints_y, map_waypoints_s,
  		   	   next_x_vals, next_y_vals);
		   break;
		case STATE_KEEP_LANE:
		   if ( prev_state == STATE_START)
		   	   target_lane = ego.lane;
		default:		   
		   generate_trajectory(target_lane,
		   	   map_waypoints_x, map_waypoints_y, map_waypoints_s,
   		   	   next_x_vals, next_y_vals);
		   break;
	}
}
void PathPlanner::generate_trajectory( int target_lane,
   vector<double> &map_waypoints_x, vector<double> &map_waypoints_y, vector<double> &map_waypoints_s,
   vector<double> &next_x_vals, vector<double> &next_y_vals)
{
	extern vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);
	float car_s = ego.s;
	// cout << "ref_vel-ego.speed: " << ref_vel - ego.speed << endl;
	int prev_size = previous_path_x.size();   
	if ( prev_size > 0 )
	{
	  car_s = end_s;
	} 
	bool too_close = false;
	int car_index = in_front_of(car_s, target_lane);
	if ( car_index != -1) {
		Vehicle front_car = vehicle_list[car_index];
	    double check_car_s = front_car.s + prev_size * 0.02 * front_car.speed;
	    if ((check_car_s > car_s) && ((check_car_s - car_s) < 22))
	    	too_close = true;
	}
	//double ref_vel = ego.speed;
	// cout << "ref_vel: " << ref_vel << endl;
	if ( too_close)
	{
		// cout << "too close" << endl;
	  if ( ref_vel > .1)
	     ref_vel -= .1;
	  else
	    ref_vel = 0;
	} 
	else {
		if ( ref_vel < 22.21) {
//	  ref_vel += .1;
	  		ref_vel += (22.21-ref_vel) * 0.0072;
	  	} else
	  	   ref_vel = 22.21;
	} 
	vector<double> ptsx;
	vector<double> ptsy;

	double ref_x = ego.x;
	double ref_y = ego.y;

	double ref_yaw = deg2rad(ego.yaw);

	if(prev_size < 2)
	{
	    //create two points defining a path tangent to the car
	    double prev_car_x = ego.x - cos(ego.yaw);
	    double prev_car_y = ego.y - sin(ego.yaw);

	    ptsx.push_back(prev_car_x);
	    ptsx.push_back(ego.x);

	    ptsy.push_back(prev_car_y);
	    ptsy.push_back(ego.y);
	}
	//use end of previous path as starting reference
	else
	{
	    ref_x = previous_path_x[prev_size-1];
	    ref_y = previous_path_y[prev_size-1];

	    double ref_x_prev = previous_path_x[prev_size-2];
	    double ref_y_prev = previous_path_y[prev_size-2];
	    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

	    ptsx.push_back(ref_x_prev);
	    ptsx.push_back(ref_x);

	    ptsy.push_back(ref_y_prev);
	    ptsy.push_back(ref_y);
	}
	vector<double> next_wp0 = getXY(car_s+30, (2+4*target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp1 = getXY(car_s+60, (2+4*target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp2 = getXY(car_s+90, (2+4*target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);

	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);

	for (int i=0; i<ptsx.size(); i++)
	{
	    double shift_x = ptsx[i]-ref_x;
	    double shift_y = ptsy[i]-ref_y;

	    ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
	    ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
	}

	for (int i=0; i<prev_size; i++)
	{
	    next_x_vals.push_back(previous_path_x[i]);
	    next_y_vals.push_back(previous_path_y[i]);
	}

	//create spline for smooth path
	tk::spline s;
	s.set_points(ptsx,ptsy);

	//calculate remaining points for path planner based on spline
	double target_x = 30.0;
	double target_y = s(target_x);
	double target_dist = sqrt(target_x*target_x + target_y*target_y);

	double N = target_dist/(.02*ref_vel);
	double dx = target_x / N;
// cout << "N, dx= " << N << "," << dx << endl;
	for(int i = 1; i <= 50-prev_size; i++) {
	    double x_point = dx * i;
	    double y_point = s(x_point);

	    double x_ref = x_point;
	    double y_ref = y_point;

	    //convert points back to global coordinates
	    x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
	    y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

	    x_point += ref_x;
	    y_point += ref_y;

	    next_x_vals.push_back(x_point);
	    next_y_vals.push_back(y_point);
	}
}

int PathPlanner::in_front_of(float s, int lane) {

  	float index = -1;
	float min_distance = 9999;
	for (int i = 0; i < vehicle_list.size(); i ++) {
	    Vehicle car = vehicle_list[i];
	    float  car_s = car.s;
	    if ( s >= (MAX_S-MAX_SENSOR_RANGE) && ( car_s < MAX_SENSOR_RANGE))
	       car_s += MAX_S;

	    if (( lane == car.lane)  && ( s < car_s )) {
	        float distance =  car_s - s;
	        if ( distance < min_distance) {
	          min_distance = distance;
	          index = i;
	        }
	    }
	}
	return index;
}
int PathPlanner::behind_of(float s, int lane) 
{
  	float index = -1;
	float min_distance = 9999;
	for (int i = 0; i < vehicle_list.size(); i ++) {
	    Vehicle car = vehicle_list[i];
	    float  car_s = car.s;
	    if ( s >= (MAX_S-MAX_SENSOR_RANGE) && (car_s < MAX_SENSOR_RANGE))
	       car_s += MAX_S;

	    if (( lane == car.lane)  && ( s > car_s)) {
	        float distance = s - car_s;
	        if ( distance < min_distance) {
	          min_distance = distance;
	          index = i;
	        }
	    }
	}
	return index;
}

double PathPlanner::speed_cost(double v)
{
	const double STOP_COST = 0.8;
	double v_cost;
	double pref_target_speed = target_speed - preferred_buffer;
	if (v <= pref_target_speed) {
		v_cost = STOP_COST * (pref_target_speed - v) / (pref_target_speed);
	}
	else if (v > target_speed)
		v_cost = 1;
	else
		v_cost = (v - pref_target_speed) / preferred_buffer;
	return v_cost * 1.5;
}
double PathPlanner::lane_speed_cost(int lane, float s, float v)
{
	int front_car_index = in_front_of(s, lane );
	if ( front_car_index != -1) {
    	float distance = vehicle_list[front_car_index].s-s;
    	if (distance < 0)
    		distance += MAX_S;
    	if ( distance > 150) // 2 second
    		return 0;
    	else
    		return speed_cost(vehicle_list[front_car_index].speed);
    } else 
       return 0;
    // int behind_car_index = behind_of(s, lane);
    // if (( front_car_index == -1) && (behind_car_index == -1))
    // 	return 0;
    // else 
    // if ( front_car_index == -1) { // no front car
    // 	float distance = s - vehicle_list[behind_car_index].s;
    // 	if (distance < 0)
    // 		distance += MAX_S;
    // 	if ( distance > 44) // 2 second
    // 		return 0;
    // 	else
    // 		return speed_cost(vehicle_list[behind_car_index].speed);
    // } 
    // else if ( behind_car_index == -1) { // no behind car
    // 	float distance = vehicle_list[front_car_index].s-s;
    // 	if (distance < 0)
    // 		distance += MAX_S;
    // 	if ( distance > 44) // 2 second
    // 		return 0;
    // 	else
    // 		return speed_cost(vehicle_list[front_car_index].speed);
    // } else { 
    // 	// check gap
    // 	float distance = vehicle_list[front_car_index].s - vehicle_list[behind_car_index].s;
    // 	if (distance < 0)
    // 		distance += MAX_S;
    // 	if ( distance < 22) // safe distance
    // 		return 9999;
    // 	else
    // 		return speed_cost(vehicle_list[behind_car_index].speed);
    // }
}
double PathPlanner::distance_cost(int lane, float s)
{
	int car_index = in_front_of(s, lane);
	if ( car_index == -1) // no car in front of
		return 0; 
	float distance = vehicle_list[car_index].s - s;
	if ( distance < 0)
		distance += MAX_S;
	if ( distance < 25)
		return 1.0;
	double d_cost = 1 - exp(-30.0 / distance);
	return d_cost;
}
bool PathPlanner::collide_prediction(int lane, float s, float speed)
{
	for (int i = 0; i < vehicle_list.size(); i ++) {
		Vehicle car = vehicle_list[i];
		if ( car.lane == lane) {
			float delta_t = 0.02;
			for ( int k = 0; k < 150; k ++) {
				float  car_s = car.s + car.speed * delta_t * i;
				float ego_s = s + ref_vel * delta_t * i;
				if ( fabs( car_s - ego_s ) < L)
				   return true;
			}
		}
	}
	return false;
}

double PathPlanner::cost_default(void)
{
  return 1; // default value for only one choice
}
double PathPlanner::cost_kl(void)
{
   return lane_speed_cost(target_lane, ego.s, ego.speed) + distance_cost(target_lane, ego.s);
}
double PathPlanner::cost_kl_plcl(void)
{
    if (target_lane == MIN_LANE_NO)
       return 9999; 
    return 0.001+ distance_cost(target_lane - 1, ego.s) + lane_speed_cost(target_lane - 1, ego.s, ego.speed);
}
double PathPlanner::cost_kl_plcr(void)
{
    if (target_lane == MAX_LANE_NO)
       return 9999; 
    return 0.001+distance_cost(target_lane + 1, ego.s) + lane_speed_cost(target_lane + 1, ego.s, ego.speed);
}
double PathPlanner::cost_plcl(void)
{
    return 0.001+distance_cost(target_lane - 1, ego.s) + lane_speed_cost(target_lane - 1, ego.s, ego.speed);
}
double PathPlanner::cost_plcl_lcl(void)
{
// check safety cost
	double cost = 0;
	if (collide_prediction(target_lane-1, ego.s, ego.speed)) {
	  // cout << "collide	predict: speed" << ego.speed << endl;
	  cost = 9999;
	}
	return cost;
}
double PathPlanner::cost_plcl_kl(void)
{
   return speed_cost(ego.speed) + distance_cost(target_lane, ego.s);
}
double PathPlanner::cost_plcr_kl(void)
{
   return speed_cost(ego.speed) + distance_cost(target_lane, ego.s);
}

double PathPlanner::cost_plcr(void)
{
    return 0.001+distance_cost(target_lane + 1, ego.s) + lane_speed_cost(target_lane + 1, ego.s, ego.speed);
}
double PathPlanner::cost_plcr_lcr(void)
{
	if (collide_prediction(target_lane+ 1, ego.s, ego.speed)) {
	  // cout << "collide	predict: speed" << ego.speed << endl;
	  return 9999;
	}
	return 0;
}
// double PathPlanner::cost_lcl(void) // check transition finished
// {

// }
// double PathPlanner::cost_lcr(void) // check transition finished
// {
	
// }
std::map<STATE_PATH_PLANNING, vector<NextStateFunc>> next_state_table = {
  { STATE_START,
    { { STATE_KEEP_LANE, &PathPlanner::cost_default , 1},
    }
  },
  { STATE_KEEP_LANE,
     { { STATE_KEEP_LANE, &PathPlanner::cost_kl, 1.},
     { STATE_PREPARE_LANE_CHANGE_LEFT, &PathPlanner::cost_kl_plcl, 1.0 },
     { STATE_PREPARE_LANE_CHANGE_RIGHT, &PathPlanner::cost_kl_plcr, 1.0 },
     }
  },
  { STATE_PREPARE_LANE_CHANGE_LEFT,
     { { STATE_PREPARE_LANE_CHANGE_LEFT, &PathPlanner::cost_plcl, 1. },
     { STATE_LANE_CHANGE_LEFT, &PathPlanner::cost_plcl_lcl, 1. },
     { STATE_KEEP_LANE, &PathPlanner::cost_plcl_kl, 1.05 },
     } 
  },
  { STATE_LANE_CHANGE_LEFT,
     { { STATE_KEEP_LANE, &PathPlanner::cost_kl, 1. },
       // { STATE_LANE_CHANGE_LEFT, &PathPlanner::cost_lcl} // in transition
     } 
  },
  { STATE_PREPARE_LANE_CHANGE_RIGHT,
     { {STATE_PREPARE_LANE_CHANGE_RIGHT, &PathPlanner::cost_plcr, 1. },
       {STATE_LANE_CHANGE_RIGHT, &PathPlanner::cost_plcr_lcr, 1. },
     { STATE_KEEP_LANE, &PathPlanner::cost_plcr_kl, 1.0 },
     } 
  },
  { STATE_LANE_CHANGE_RIGHT,
    { { STATE_KEEP_LANE, &PathPlanner::cost_kl, 1 },
       // { STATE_LANE_CHANGE_RIGHT, &PathPlanner::cost_lcr} // in transition
    }
  }
};  

void PathPlanner::updateState(void)
{ if (target_lane < 0) // invalid or intial value
	 target_lane = ego.lane;
  if (target_lane != ego.lane) { // wait ego go to target lane before change state
  	 prev_state = state;
     return; // remain current state
  }
  vector<NextStateFunc> next_state_list = next_state_table[state];
  double min_cost = 99999;
  int min_index;
  //cout << "current lane, s, v, a: " << this->lane << ", " << this->s << ", " << this->v << ", " << this->a << endl;
  for (int i = 0; i < next_state_list.size(); i++) {
    cost_function costf = next_state_list[i].func;
//    double score = std::invoke( costf, this, predictions); // call member function
    double score = (this->*costf)() * next_state_list[i].weight;
    cout << " state, score: " << next_state_list[i].state << ", " << score << endl;
    if (score < min_cost) {
      min_cost = score;
      min_index = i;
    }
  }
  this->prev_state = this->state;
  this->state = next_state_list[min_index].state; // this is an example of how you change state.
}

