#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

using namespace std;
class Vehicle;
typedef double (Vehicle::*cost_function)(map<int, vector < vector<int> >> predictions);
typedef struct {
	string state;
	cost_function func;
} NextStateFunc;

typedef struct {
   double x, y, vx, vy;
   duoble s, d;
} VehicleStateValue;
 
typedef struct {
   int lane;
   double s;
} PredictionValue;
typedef vector<PredictionValue> Predictions;

class Vehicle {
public:

  struct collider{

    bool collision ; // is there a collision?
    int  time; // time collision happens

  };
  
  int L = 1;

  int preferred_buffer = 6; // impacts "keep lane" behavior.
  VehicleStateValue stateValue;
  int lane;

  int s;

  int v;

  int a;

  int target_speed;

  int lanes_available;

  int max_acceleration;

  int goal_lane;

  int goal_s;

  string state;

  /**
  * Constructor
  */
  Vehicle(int lane, int s, int v, int a);
  Vehicle(VehicleStateValue aState);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  void update_state(map<int, vector <vector<int> > > predictions);

  void configure(vector<int> road_data);

  void increment(int dt);

  vector<int> state_at(int t);

  bool collides_with(Vehicle other, int at_time);

  collider will_collide_with(Vehicle other, int timesteps);

  void realize_state(map<int, vector < vector<int> > > predictions);

  void realize_constant_speed();

  int _max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s);

  void realize_keep_lane(map<int, vector< vector<int> > > predictions);

  void realize_lane_change(map<int,vector< vector<int> > > predictions, string direction);

  void realize_prep_lane_change(map<int,vector< vector<int> > > predictions, string direction);

  vector<vector<int> > generate_predictions(int horizon);

  /* private static */
  
  static std::map<string, vector<string>> next_state_table0;
  static std::map<string, vector<NextStateFunc>> next_state_table;
//  double cost(string next_state, map<int, vector < vector<int> > > predictions);
  double cost_default(map<int, vector < vector<int> > > predictions);
  double cost_kl(map<int, vector < vector<int> > > predictions);
  double cost_kl_plcl(map<int, vector < vector<int> > > predictions);
  double cost_kl_plcr(map<int, vector < vector<int> > > predictions);
  double cost_plcl(map<int, vector < vector<int> > > predictions);
  double cost_plcl_kl(map<int, vector < vector<int> > > predictions);
  double cost_plcl_lcl(map<int, vector < vector<int> > > predictions);
  double cost_plcr(map<int, vector < vector<int> > > predictions);
  double cost_plcr_kl(map<int, vector < vector<int> > > predictions);
  double cost_plcr_lcr(map<int, vector < vector<int> > > predictions);
  double SAFTTY_COST = 1000;
  double distance_cost(int lane, int s);
  double speed_cost(double v);
  bool collide_prediction(int lane, int s, int delta_t, map<int, vector < vector<int> > > predictions);
  double lane_speed_cost(int lane, map<int, vector < vector<int> > > predictions);
};

#endif