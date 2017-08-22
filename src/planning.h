
class Vehicle {
//  int preferred_buffer = 6; // impacts "keep lane" behavior.
//  VehicleStateValue stateValue;

public:
  int id;
  float x, y, vx, vy, s, d; // data from fusion sensor[1], 2, 3, 4, 5, 6
  int lane;
  float yaw; // degree
  float speed; // mph
  void updateData( float x, float y, float vx, float vy, float s, float d);
  Vehicle(int id, float x, float y, float vx, float vy, float s, float d);
  Vehicle();
};
class EgoVehicle: public Vehicle {
public:
   EgoVehicle(float x, float y, float s, float d, float yaw, float speed);
   EgoVehicle();
   void updateData(float x, float y, float s, float d, float yaw, float speed);
};
typedef enum {
  STATE_START = 0, 
  STATE_KEEP_LANE = 1, 
  STATE_PREPARE_LANE_CHANGE_LEFT = 2, 
  STATE_LANE_CHANGE_LEFT = 3,
  STATE_PREPARE_LANE_CHANGE_RIGHT = 4, 
  STATE_LANE_CHANGE_RIGHT= 5
} STATE_PATH_PLANNING;

class PathPlanner;
typedef double (PathPlanner::*cost_function)(void);
typedef struct {
   STATE_PATH_PLANNING state;
   cost_function func;
   double weight;
} NextStateFunc;

class PathPlanner {
public:
  vector<Vehicle> vehicle_list;
  EgoVehicle ego;
  STATE_PATH_PLANNING state = STATE_START;
  STATE_PATH_PLANNING prev_state = STATE_START;
  int target_lane = -1;
  double ref_vel = 0.0;
  const float L = 10; 
  float target_speed;
  float preferred_buffer = 0.8; 
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  double end_s;
  double end_d;
  void updateFusionSensor(vector<vector<float>> &sensor_fusion);
  void updateEgoCar(float x, float y, float s, float d, float yaw, float speed);
  int in_front_of(float s, int lane);
  int behind_of(float s, int lane);
  PathPlanner();
  double cost_default(void);
  double cost_kl(void);
  double cost_kl_plcl(void);
  double cost_kl_plcr(void);
  double cost_plcl(void);
  double cost_plcl_kl(void);
  double cost_plcr_kl(void);
  double cost_plcr(void);
  double cost_plcl_lcl(void);
  double cost_plcr_lcr(void);

  // double cost_lcl(void);
  // double cost_lcr(void);
  void updateState(void); 
  void updatePath(vector<double> previous_path_x, vector<double> previous_path_y, double end_s, double end_d);
  void realizeState(
    vector<double> &map_waypoints_x, vector<double> &map_waypoints_y, vector<double> &map_waypoints_s,
    vector<double> &next_x_vals, vector<double> &next_y_vals);  
private:  
  double speed_cost(double v);
  double distance_cost(int lane, float s);
  double lane_speed_cost(int lane, float s, float v);
  bool collide_prediction(int lane, float s, float speed);
  void generate_trajectory( int target_lane,
          vector<double> &map_waypoints_x, vector<double> &map_waypoints_y, vector<double> &map_waypoints_s,
          vector<double> &next_x_vals, vector<double> &next_y_vals);
};