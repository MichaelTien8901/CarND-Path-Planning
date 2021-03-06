#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include "json.hpp"
#include "spline.h"
#ifdef DEBUG_LOG
#include <signal.h>
#endif

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
#include "planning.h"
// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
constexpr double deg2rad(double x) { return x * pi() / 180; }
constexpr double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

vector<double> JMT(vector< double> start, vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
    MatrixXd m = MatrixXd(3, 3);
    double T2 = T*T;
    double T3 = T2*T;
    double T4 = T3*T;
    double T5 = T4*T;
    m << T3, T4, T5, 3.0*T2, 4.0*T3, 5.0*T4, 6.0*T, 12.0*T2, 20.0 * T3;
    MatrixXd m_inverse = m.inverse();
    VectorXd s = VectorXd(3);
    s(0) = end[0] - (start[0] + start[1] * T + 0.5 * start[2] * T2);
    s(1) = end[1] - (start[1] + start[2] * T);
    s(2) = end[2] - start[2];
    VectorXd a = VectorXd(3);
    a = m_inverse * s;
    return {start[0], start[1], 0.5 *start[2], a(0), a(1), a(2)};
}
double f_t(vector<double> alpha, double t)
{
  int order = alpha.size();
  double t1 = 1;     
  double f = alpha[0];
  for (int i = 1; i < order; i ++) {
      t1 *= t;
      f += alpha[i] * t1;
  }
  return f;
}
double dot_f_t(vector<double> alpha, double t)
{
  int order = alpha.size();
  double t1 = 1;     
  double f = alpha[1];
  for (double i = 2.; i < order; i += 1.0) {
      t1 *= t;
      f += alpha[i] * t1 * i;
  }
  return f;
}
double dot_dot_f_t(vector<double> alpha, double t)
{
  int order = alpha.size();
  double t1 = 1;     
  double f = alpha[2] * 2.0;
  for (double i = 3.; i < order; i += 1.0) {
    t1 *= t;
    f += alpha[i] * t1 * i *(i-1);
  }
  return f;
}
bool check_max_acceleration_safe(vector<double> alpha, double t, double delta_t, double max_value) 
{
  for ( double t1 = 0; t1 < t; t1 += delta_t) {
    if ( dot_dot_f_t(alpha, t1) > max_value) {
      return false;
    }
  }
  return true;
}

bool check_velocity_safe(vector<double> alpha, double t, double delta_t, double min_value, double max_value) 
{
  for ( double t1 = 0; t1 < t; t1 += delta_t) {
    double v = dot_f_t(alpha, t1);
    if ( (v > max_value) || (v < min_value)) {
      return false;
    }
  }
  return true;
}
bool check_collision_safe(vector<double> alpha, double t, double delta_t, double value, double tolerance) 
{
  for ( double t1 = 0; t1 < t; t1 += delta_t) {
    if ( fabs(f_t(alpha, t1)-value) < tolerance) 
      return false;
  }
  return true; 
}
#define LANE_WIDTH 4.0
#define MPH_to_MPS(x) (x*0.44074)

#ifdef DEBUG_LOG
ofstream debug_output;
void my_signal_handler(int s)
{
  cout << "SIGINT caughted." << endl;
  debug_output.close();  
  exit(1);
}
#endif


int main() {
#ifdef DEBUG_LOG
  signal(SIGINT, my_signal_handler);
  debug_output.open("debug_log.txt");
#endif
  uWS::Hub h;
//  chrono::high_resolution_clock::time_point next_time_count = chrono::high_resolution_clock::now();

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0


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
  PathPlanner path_planner;

  // // use spline to interpolate with high resolution
  // tk::spline sx;
  // tk::spline sy;
  // sx.set_points(map_waypoints_s, map_waypoints_x);   
  // sy.set_points(map_waypoints_s, map_waypoints_y);

  // vector<double> map_waypoints_x1;
  // vector<double> map_waypoints_y1;
  // vector<double> map_waypoints_s1;
  // // total s = last s + last point to first point
  // double total_s = map_waypoints_s.back() + 
  //   distance(map_waypoints_x.back(), map_waypoints_y.back(), map_waypoints_x[0], map_waypoints_y[0]);
  // for(double s = 0; s < total_s; s+= 1.0) { // resolution 1 meter(s)
  //   cout << "s, sx, sy = " << s << "," << sx(s) << "," << sy(s) << endl;
  //   map_waypoints_x1.push_back(sx(s));
  //   map_waypoints_y1.push_back(sy(s));
  //   map_waypoints_s1.push_back(s);
  // }
//  double ref_vel = 0.0;
//  int lane = 1;
  h.onMessage([
       &map_waypoints_x,
       &map_waypoints_y,
       &map_waypoints_s,
       &map_waypoints_dx,
       &map_waypoints_dy,
       // &next_time_count,
       &path_planner
          ](
          uWS::WebSocket<uWS::SERVER> ws, 
          char *data, size_t length,
          uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {

          // j[1] is the data JSON object
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];
            //cout << "car_x, car_y = " << car_x << "," << car_y << endl;
            path_planner.updateEgoCar(car_x, car_y, car_s, car_d, car_yaw, car_speed);
          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];
            path_planner.updatePath(previous_path_x, previous_path_y, end_path_s, end_path_d);
          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

                // do planning

            // update sensor fusion data
            vector<vector<float>> sensor_fusion_list;
            for(int i = 0; i < sensor_fusion.size(); i++)
            {
              vector<float> car_data;
              for( int j = 0; j < 7; j++) {
                 car_data.push_back(sensor_fusion[i][j]);
              }
              sensor_fusion_list.push_back(car_data);
            }
            path_planner.updateFusionSensor(sensor_fusion_list);

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            // const int PLANNING_PERIOD = 800; // ms
            // const double delta_t = 0.02; // 20ms
            path_planner.updateState();
            path_planner.realizeState(
              map_waypoints_x, map_waypoints_y, map_waypoints_s,
              next_x_vals, next_y_vals);

            // chrono::high_resolution_clock::time_point current_time_count = chrono::high_resolution_clock::now(); //
            // if ( current_time_count >= next_time_count) {
            //     next_time_count = current_time_count + chrono::milliseconds(PLANNING_PERIOD);
            //     // do planning
            // } else {
            // }

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
















































































