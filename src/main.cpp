#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <ctime>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
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

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
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
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int next_waypoint = NextWaypoint(x,y, theta, maps_x,maps_y);

    int prev_wp;
    prev_wp = next_waypoint-1;
    if(next_waypoint == 0)
    {
        prev_wp  = maps_x.size()-1;
    }

    double n_x = maps_x[next_waypoint]-maps_x[prev_wp];
    double n_y = maps_y[next_waypoint]-maps_y[prev_wp];
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
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
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

float distFromVelTimeSpp(double s, double vel_x, double vel_y, double dt, int spp)
{
					double speed = sqrt(vel_x * vel_x + vel_y * vel_y);
					double pred_s = s + (double)spp * speed * dt;	
					return pred_s;
}

// initial values needed outside the main loop
int car_lane = 1;
int t_start_left_lane;
int t_switch_lane = 0;
bool debug = true;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

            // Previous path data given to the Planner
            auto previous_path_x = j[1]["previous_path_x"];
            auto previous_path_y = j[1]["previous_path_y"];
            // Previous path's end s and d values 
            double end_path_s = j[1]["end_path_s"];
            double end_path_d = j[1]["end_path_d"];

            // Sensor Fusion Data, a list of all other cars on the same side of the road.
            auto sensor_fusion = j[1]["sensor_fusion"];
			
			///////////////////////////////
            // 0 - INITIAL VALUES IN LOOP//
			///////////////////////////////
            double desired_speed_mph = 49.3; // less than 50mph to not violate the speed limit transiently
			
			/////////////////////////////////////////////////////////////////
            // 1 - PREDICT S, SPEED AT THE END OF THE RETURNED UNUSED PATH //
			/////////////////////////////////////////////////////////////////            
            int spp = previous_path_x.size();
            double sensor_car_x = car_x;
            double sensor_car_y = car_y;
            double sensor_car_yaw = deg2rad(car_yaw);            
            if (2 <= spp)
            {
				// use the last 2 points in the path to "predict" yaw and speed
                sensor_car_x = previous_path_x[spp-1];
                sensor_car_y = previous_path_y[spp-1];
                double sensor_car_x_prev = previous_path_x[spp-2];
                double sensor_car_y_prev = previous_path_y[spp-2];
                sensor_car_yaw = atan2(sensor_car_y - sensor_car_y_prev, sensor_car_x - sensor_car_x_prev);
                car_speed = (sqrt((sensor_car_x-sensor_car_x_prev) * (sensor_car_x-sensor_car_x_prev)+
                                  (sensor_car_y-sensor_car_y_prev) * (sensor_car_y-sensor_car_y_prev)) / .02) * 2.23694;
				// "predict" s
				car_s = end_path_s;
            }
						
			///////////////////////
            // 2 - FINITE STATES //
			///////////////////////           
            double min_dist_s = 99999999.9;
			double car_front_speed;
            bool switch_lane = false;
			bool car_front_exists = false;
			
			// search thru other cars from sensor fusion
            for (int i = 0; i < sensor_fusion.size(); i++)
            {
				// if in my lane
				if (  4*car_lane < sensor_fusion[i][6] && sensor_fusion[i][6] < (4 * car_lane + 4) )
				{
					// predict other car s
                    double sensed_car_speed = sqrt((double)sensor_fusion[i][3] * (double)sensor_fusion[i][3] + 
												   (double)sensor_fusion[i][4] * (double)sensor_fusion[i][4]);
                    double sensed_car_s = (double)sensor_fusion[i][5] + (double)spp * sensed_car_speed * 0.02;
					double dist_s = sensed_car_s - car_s;
					// find closest car in front
					if (car_s < sensed_car_s && dist_s < min_dist_s) 
					{			
						car_front_exists = true;
						min_dist_s = dist_s;
						car_front_speed = sensed_car_speed;
					}
				}			
			}

			// STATE - car in front => switch lane
			if (car_front_exists && min_dist_s < 25.0)
			{
				double min_spd = car_front_speed * 2.23694 - 3.5;
				double max_spd = car_front_speed * 2.23694;
				if (min_dist_s < +25.0) desired_speed_mph = min_spd + (max_spd - min_spd)/(25.0 - 12.5)*(min_dist_s - 12.5);
				if (min_dist_s < +12.5) desired_speed_mph = min_spd;
				switch_lane = true;
			}
			// STATE - no car in front => cruise
			if (!car_front_exists)
			{
				desired_speed_mph = desired_speed_mph;
				switch_lane = false ;
			}	
			// STATE - on left most lane => switch lane
			if (car_lane ==0 && (t_start_left_lane + 7 < time(nullptr)) )
			{
				desired_speed_mph = desired_speed_mph;
				switch_lane = true;
			}
			
			if (debug) cout<<" car_front_exists="<<car_front_exists << " min_dist_s="<<min_dist_s<<" car_lane="<<car_lane<<" switch_lane="<<switch_lane<<endl;
			
			//////////////////////////////////////
            // 3 - SAFETY CHECK FOR SWITCH LANE //
			//////////////////////////////////////

			// if state is to switch lane and 3 secs have passed since last switch lane
            if (switch_lane  &&  3 < time(nullptr) - t_switch_lane)
            {
				bool safety_left;
				bool safety_right;
				if (car_lane == 0) safety_left  = false; else safety_left  = true; 
				if (car_lane == 2) safety_right = false; else safety_right = true; 
				// search thru other cars from sensor fusion 
				for (int i = 0; i < sensor_fusion.size(); i++)
				{
					// predict other car s
                    double sensed_car_speed = sqrt((double)sensor_fusion[i][3] * (double)sensor_fusion[i][3] + 
												   (double)sensor_fusion[i][4] * (double)sensor_fusion[i][4]);
                    double sensed_car_s = (double)sensor_fusion[i][5] + (double)spp * sensed_car_speed * 0.02;
					float  sensed_car_d = sensor_fusion[i][6];
					double dist_s = sensed_car_s - car_s;
					// range of unsafe distance to closest cars
					if(-18.0 < dist_s && dist_s < +22.0)
					{
						// if on adjacent lanes and on distance range => unsafe to switch lanes
						if (car_lane != 0 && 4 * car_lane - 4 < sensed_car_d && sensed_car_d < 4 * car_lane)     safety_left  = false;
						if (car_lane != 2 && 4 * car_lane + 4 < sensed_car_d && sensed_car_d < 4 * car_lane + 8) safety_right = false;
					} 
				}
				// if safe to switch lanes:
				// substract/add to lane to determine new lane
				// start time counters
				if (safety_left)
				{
					car_lane = car_lane - 1;
					t_switch_lane = time(nullptr);
					if (car_lane == 0) t_start_left_lane = time(nullptr);
				}
				if (!safety_left && safety_right)
				{
					car_lane = car_lane + 1;
					t_switch_lane = time(nullptr);
				}
			if (debug) cout<<"\t\t\t\t\t\t\t\t safety_left="<<safety_left<<" safety_right="<<safety_right<<endl;
            }
			
			
			///////////////////////////////
            // 4 - TRAJECTORY GENERATION //
			///////////////////////////////

			// using the current car_lane and s compute 3 XY locations at 27m, 54m, and 81m
            vector<double> sXY27m = getXY(car_s + 27, 2 + 4 * car_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> sXY54m = getXY(car_s + 54, 2 + 4 * car_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> sXY81m = getXY(car_s + 81, 2 + 4 * car_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            // create X and Y vectors for using the last 2 unused points on path + 3 sXY points
            vector<double> path_x;
            vector<double> path_y;
			double car_speed_tmp;
            if (2 <= spp)
            {
				path_x = {previous_path_x[spp-2], previous_path_x[spp-1], sXY27m[0], sXY54m[0], sXY81m[0]};
				path_y = {previous_path_y[spp-2], previous_path_y[spp-1], sXY27m[1], sXY54m[1], sXY81m[1]};
			}
			else
			{
				if (car_speed < 0.1) car_speed_tmp = 0.1;
				else 			     car_speed_tmp = car_speed;
				// transform w.r.t car C.S.
				path_x = {car_x - car_speed_tmp * 0.02 * cos(car_yaw), car_x, sXY27m[0], sXY54m[0], sXY81m[0]};
				path_y = {car_y - car_speed_tmp * 0.02 * sin(car_yaw), car_y, sXY27m[1], sXY54m[1], sXY81m[1]};				
			}
            for (int i = 0; i < path_x.size(); i++ )
            {
                double x_wrt_sensor = path_x[i] - sensor_car_x;
                double y_wrt_sensor = path_y[i] - sensor_car_y;
				// transform w.r.t car C.S.
                path_x[i] = x_wrt_sensor * cos(-sensor_car_yaw) - y_wrt_sensor * sin(-sensor_car_yaw);
                path_y[i] = x_wrt_sensor * sin(-sensor_car_yaw) + y_wrt_sensor * cos(-sensor_car_yaw);
            }
            
			// create spline
            tk::spline s;
            s.set_points(path_x,path_y);
            double s_x = 27.0;
			// find the value of Y at 27m on spline (w.r.t car C.S.)
            double s_y = s(s_x);
			// approximate length of spline by linearizing it
            double s_dist = sqrt(s_x * s_x + s_y * s_y);
            double x_point_prev = 0;

			// initialize next_ vector to be sent to simulator
			// use unused path if exists
            vector<double> next_x_vals, next_y_vals;
            for(int i = 0; i < previous_path_x.size(); i++)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }			
			// add remaining points to total of 50 using spline
            for (int i = 1; i <= 50 - previous_path_x.size(); i++) 
            {
				// increase/decrease speed according to state
                if(desired_speed_mph > car_speed)      car_speed = car_speed + 0.20;
                else if(desired_speed_mph < car_speed) car_speed = car_speed - 0.20;
				// divide S by traveled distance every 0.02sec
                double N_points_on_S = s_dist / (.02 * car_speed / 2.23694);
				// compute x linearized
                double x_point = x_point_prev + s_x / N_points_on_S;
				// get spline value for x
                double y_point = s(x_point);
                x_point_prev = x_point;
				// transform w.r.t global C.S.
                double x_point_tmp = x_point;
                double y_point_tmp = y_point;
                x_point = (x_point_tmp * cos(sensor_car_yaw) - y_point_tmp * sin(sensor_car_yaw)) + sensor_car_x;
                y_point = (x_point_tmp * sin(sensor_car_yaw) + y_point_tmp * cos(sensor_car_yaw)) + sensor_car_y;
				// put into next_ vector to be sent to simulator
                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
            }
            
            json msgJson;
            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;
            auto msg = "42[\"control\","+ msgJson.dump()+"]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            
            // GOING ON A STRAIGHT LINE AT 25m/s
/*          double dist_inc = 0.5;
            for(int i=0; i< 50;i++){
                next_x_vals.push_back(car_x + (dist_inc * i * cos(deg2rad(car_yaw))));
                next_y_vals.push_back(car_y + (dist_inc * i * sin(deg2rad(car_yaw))));
            } */

            // GOING IN CIRCLES
/*          double pos_x;
            double pos_y;
            double angle;
            int path_size = previous_path_x.size(); cout << "path size=" << path_size <<endl;
            for (int i=0;i<path_size;i++){
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }
            if (path_size==0){
                pos_x = car_x;
                pos_y = car_y;
                angle = deg2rad(car_yaw);
            }
            else{
                pos_x = previous_path_x[path_size-1];
                pos_y = previous_path_y[path_size-1];
                double pos_x2 = previous_path_x[path_size-2];
                double pos_y2 = previous_path_y[path_size-2];
                angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
            }
            double dist_inc = 0.5;
            for(int i=0; i< 50-path_size;i++){
                next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
                next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
                pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
                pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
            }    */     
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
