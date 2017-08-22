#include <iostream>
#include "vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32)
#define min(x,y) ((x)>(y)?(y):(x)) 
#define max(x,y) ((x)>(y)?(x):(y))
#endif

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int lane, int s, int v, int a) {

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    state = "CS";
    max_acceleration = -1;
}
Vehicle::~Vehicle() {}

// TODO - Implement this method.
std::map<string, vector<NextStateFunc>> Vehicle::next_state_table = {
	{ "CS",
	  { { "KL", &Vehicle::cost_default }
	  }
	},
	{ "KL",
	   { { "KL", &Vehicle::cost_kl },
		 { "PLCL", &Vehicle::cost_kl_plcl },
		 { "PLCR", &Vehicle::cost_kl_plcr }
	   }
	},
	{ "PLCL",
	   { { "PLCL", &Vehicle::cost_plcl },
		 { "LCL", &Vehicle::cost_plcl_lcl },
		 { "KL", &Vehicle::cost_plcl_kl }
	   } 
	},
	{ "LCL",
	   { { "KL", &Vehicle::cost_kl },
	     { "PLCL", &Vehicle::cost_kl_plcl },
	     { "PLCR", &Vehicle::cost_kl_plcr }
	   }
	},
	{ "PLCR",
	   { {"PLCR", &Vehicle::cost_plcr },
		 {"LCR", &Vehicle::cost_plcr_lcr },
		 {"KL", &Vehicle::cost_plcr_kl }
	   } 
	},
	{ "LCR",
	  { { "KL", &Vehicle::cost_kl },
        { "PLCL", &Vehicle::cost_kl_plcl },
        { "PLCR", &Vehicle::cost_kl_plcr }
	  }
	}
};
double Vehicle::lane_speed_cost(int lane, map<int, vector < vector<int> > > predictions)
{
	map<int, vector<vector<int> > >::iterator it = predictions.begin();
	vector<vector<vector<int> > > at_behind;
	while (it != predictions.end())
	{
		int v_id = it->first;
		vector<vector<int> > v = it->second;

		if ((v[0][0] == lane) && (v[0][1] <= this->s))
		{
			at_behind.push_back(v);
		}
		it++;
	}
	double vcost = 0;
	if (at_behind.size() > 0)
	{

		int max_s = -1000;
		vector<vector<int> > nearest_behind = {};
		for (int i = 0; i < at_behind.size(); i++)
		{
			if ((at_behind[i][0][1]) > max_s)
			{
				max_s = at_behind[i][0][1];
				nearest_behind = at_behind[i];
			}
		}
		int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
		vcost = speed_cost(target_vel);
	} 
	return vcost;
}

double Vehicle::distance_cost(int lane, int s)
{
	double d_cost = 1 - exp(-fabs(goal_lane - lane) * 10.0 / (goal_s - s));
	return d_cost * 2;
}
double Vehicle::speed_cost(double v)
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
	return v_cost;
}
double Vehicle::cost_default(map<int, vector < vector<int> > > predictions)
{
	return 1; // default value for only one choice
}
double Vehicle::cost_kl(map<int, vector < vector<int> > > predictions)
{
	return speed_cost(this->v) + distance_cost(this->lane, this->s);
}
double Vehicle::cost_kl_plcl(map<int, vector < vector<int> > > predictions)
{
	if (this->lane == (lanes_available-1))
		return SAFTTY_COST;

	return distance_cost(this->lane + 1, this->s) + lane_speed_cost(this->lane + 1, predictions);
}
double Vehicle::cost_kl_plcr(map<int, vector < vector<int> > > predictions)
{
	if (this->lane == 0)
		return SAFTTY_COST;
	return distance_cost(this->lane-1, this->s) + lane_speed_cost(this->lane - 1, predictions); 
}
double Vehicle::cost_plcl(map<int, vector < vector<int> > > predictions)
{
	return distance_cost(this->lane + 1, this->s) + lane_speed_cost(this->lane + 1, predictions);
}
double Vehicle::cost_plcl_kl(map<int, vector < vector<int> > > predictions)
{
	return distance_cost(this->lane, this->s) + speed_cost(this->v);
}
double Vehicle::cost_plcr_kl(map<int, vector < vector<int> > > predictions)
{
	return distance_cost(this->lane, this->s) + speed_cost(this->v);
}

double Vehicle::cost_plcr(map<int, vector < vector<int> > > predictions)
{
	// keep plcr
	return distance_cost(this->lane - 1, this->s) + lane_speed_cost(this->lane - 1, predictions);

}
double Vehicle::cost_plcl_lcl(map<int, vector < vector<int> > > predictions)
{
	// check safety cost
	double cost = 0;
	if (collide_prediction(this->lane - 1, this->s, 0, predictions))
		cost += SAFTTY_COST;
	return cost;
}
double Vehicle::cost_plcr_lcr(map<int, vector < vector<int> > > predictions)
{
	// check safety cost
	double cost = 0;
	if (collide_prediction(this->lane + 1, this->s, 0, predictions))
		cost += SAFTTY_COST;	 
	return cost;
}

void Vehicle::update_state(map<int,vector < vector<int> > > predictions) {
	/*
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is 
       traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
     - The vehicle will find the nearest vehicle in the adjacent lane which is
       BEHIND itself and will adjust speed to try to get behind that vehicle.

    INPUTS
    - predictions 
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the 
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
      3 : [
        {"s" : 4, "lane": 0},
        {"s" : 6, "lane": 0},
        {"s" : 8, "lane": 0},
        {"s" : 10, "lane": 0},
      ]
    }

    */
	vector<NextStateFunc> next_state_list = next_state_table[state];
	double min_cost = 99999;
	int min_index;
	//cout << "current lane, s, v, a: " << this->lane << ", " << this->s << ", " << this->v << ", " << this->a << endl;
	for (int i = 0; i < next_state_list.size(); i++) {
		cost_function costf = next_state_list[i].func;
//		double score = std::invoke( costf, this, predictions); // call member function
		double score = (this->*costf)(predictions);
		//cout << " state, score: " << next_state_list[i].state << ", " << score << endl;
		if (score < min_cost) {
			min_cost = score;
			min_index = i;
		}
	}
	//cout << "** Next state, cost: " << next_state_list[min_index].state << ", " << min_cost << endl;
    this->state = next_state_list[min_index].state; // this is an example of how you change state.
}

void Vehicle::configure(vector<int> road_data) {
	/*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle. 
    */
    target_speed = road_data[0];
    lanes_available = road_data[1];
    goal_lane = road_data[3];
    goal_s = road_data[2];
	max_acceleration = road_data[4];
}

string Vehicle::display() {

	ostringstream oss;
	
	oss << "s:    " << this->s << "\n";
    oss << "lane: " << this->lane << "\n";
    oss << "v:    " << this->v << "\n";
    oss << "a:    " << this->a << "\n";
    
    return oss.str();
}

void Vehicle::increment(int dt = 1) {

	this->s += this->v * dt;
    this->v += this->a * dt;
}
bool Vehicle::collide_prediction(int lane, int s, int delta_t, map<int, vector < vector<int> > > predictions)
{
	map<int, vector<vector<int> > >::iterator it = predictions.begin();
	vector<vector<vector<int> > > at_behind;
	while (it != predictions.end())
	{
		int v_id = it->first;
		vector<vector<int> > v = it->second;

		if ((v[0][0] == lane) && (abs(v[0][1]-s) <= L))
		{
			return true;
		}
		it++;
	}
	return false;
}
vector<int> Vehicle::state_at(int t) {

	/*
    Predicts state of vehicle in t seconds (assuming constant acceleration)
    */
    int s = this->s + this->v * t + this->a * t * t / 2;
    int v = this->v + this->a * t;
    return {this->lane, s, v, this->a};
}

bool Vehicle::collides_with(Vehicle other, int at_time) {

	/*
    Simple collision detection.
    */
    vector<int> check1 = state_at(at_time);
    vector<int> check2 = other.state_at(at_time);
    return (check1[0] == check2[0]) && (abs(check1[1]-check2[1]) <= L);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {

	Vehicle::collider collider_temp;
	collider_temp.collision = false;
	collider_temp.time = -1; 

	for (int t = 0; t < timesteps+1; t++)
	{
      	if( collides_with(other, t) )
      	{
			collider_temp.collision = true;
			collider_temp.time = t; 
        	return collider_temp;
    	}
	}

	return collider_temp;
}

void Vehicle::realize_state(map<int,vector < vector<int> > > predictions) {
   
	/*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
    string state = this->state;
    if(state.compare("CS") == 0)
    {
    	realize_constant_speed();
    }
    else if(state.compare("KL") == 0)
    {
    	realize_keep_lane(predictions);
    }
    else if(state.compare("LCL") == 0)
    {
    	realize_lane_change(predictions, "L");
    }
    else if(state.compare("LCR") == 0)
    {
    	realize_lane_change(predictions, "R");
    }
    else if(state.compare("PLCL") == 0)
    {
    	realize_prep_lane_change(predictions, "L");
    }
    else if(state.compare("PLCR") == 0)
    {
    	realize_prep_lane_change(predictions, "R");
    }

}

void Vehicle::realize_constant_speed() {
	a = 0;
}

int Vehicle::_max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s) {

	int delta_v_til_target = target_speed - v;
    int max_acc = min(max_acceleration, delta_v_til_target);

    map<int, vector<vector<int> > >::iterator it = predictions.begin();
    vector<vector<vector<int> > > in_front;
    while(it != predictions.end())
    {
       
    	int v_id = it->first;
    	
        vector<vector<int> > v = it->second;
        
        if((v[0][0] == lane) && (v[0][1] > s))
        {
        	in_front.push_back(v);

        }
        it++;
    }
    
    if(in_front.size() > 0)
    {
    	int min_s = 1000;
    	vector<vector<int>> leading = {};
    	for(int i = 0; i < in_front.size(); i++)
    	{
    		if((in_front[i][0][1]-s) < min_s)
    		{
    			min_s = (in_front[i][0][1]-s);
    			leading = in_front[i];
    		}
    	}
    	
    	int next_pos = leading[1][1];
    	int my_next = s + this->v;
    	int separation_next = next_pos - my_next;
    	int available_room = separation_next - preferred_buffer;
    	max_acc = min(max_acc, available_room);
    }
    
    return max_acc;

}

void Vehicle::realize_keep_lane(map<int,vector< vector<int> > > predictions) {
	this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int,vector< vector<int> > > predictions, string direction) {
	int delta = -1;
    if (direction.compare("L") == 0)
    {
    	delta = 1;
    }
    this->lane += delta;
    int lane = this->lane;
    int s = this->s;
    this->a = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_prep_lane_change(map<int,vector<vector<int> > > predictions, string direction) {
	int delta = -1;
    if (direction.compare("L") == 0)
    {
    	delta = 1;
    }
    int lane = this->lane + delta;

    map<int, vector<vector<int> > >::iterator it = predictions.begin();
    vector<vector<vector<int> > > at_behind;
    while(it != predictions.end())
    {
    	int v_id = it->first;
        vector<vector<int> > v = it->second;

        if((v[0][0] == lane) && (v[0][1] <= this->s))
        {
        	at_behind.push_back(v);

        }
        it++;
    }
    if(at_behind.size() > 0)
    {

    	int max_s = -1000;
    	vector<vector<int> > nearest_behind = {};
    	for(int i = 0; i < at_behind.size(); i++)
    	{
    		if((at_behind[i][0][1]) > max_s)
    		{
    			max_s = at_behind[i][0][1];
    			nearest_behind = at_behind[i];
    		}
    	}
    	int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
    	int delta_v = this->v - target_vel;
    	int delta_s = this->s - nearest_behind[0][1];
    	if(delta_v != 0)
    	{

    		int time = -2 * delta_s/delta_v;
    		int a;
    		if (time == 0)
    		{
    			a = this->a;
    		}
    		else
    		{
    			a = delta_v/time;
    		}
    		if(a > this->max_acceleration)
    		{
    			a = this->max_acceleration;
    		}
    		if(a < -this->max_acceleration)
    		{
    			a = -this->max_acceleration;
    		}
    		this->a = a;
    	}
    	else
    	{
    		int my_min_acc = max(-this->max_acceleration,-delta_s);
    		this->a = my_min_acc;
    	}

    }

}

vector<vector<int> > Vehicle::generate_predictions(int horizon = 10) {

	vector<vector<int> > predictions;
    for( int i = 0; i < horizon; i++)
    {
      vector<int> check1 = state_at(i);
      vector<int> lane_s = {check1[0], check1[1]};
      predictions.push_back(lane_s);
  	}
    return predictions;

}