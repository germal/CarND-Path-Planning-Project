/*                                                                         80->|
* main.cpp
*
* Modifications: James William Dunn
*          Date: August 26, 2017
*/

#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "timer.h"
#include "spline.h"
#include <string>

using namespace std;
#define VELOCITYLOW     17.8816   // 40mph - low cruise
#define VELOCITYHIGH    21.905    // 49mph - high cruise
#define MAXLANEDEPTH    99999.0
#define QDEPTH          50        // the queue length to send to the simulator
#define DRIFTTOLERANCE  3.26      // obstacle delta d minimum
#define NORMALACCEL     0.002
#define STEERINGMAX     0.005
#define MAX_S           6945.554  // the max s value before wrap to 0
#define TARGETPOINT     12.5      // the distance ahead on the spline
#define RADIUSSLC       500.0     // threshold for single lane change
#define RADIUSDLC       3000.0    // threshold for double "

#define T5SECONDS       5000      // for readability in the rules
#define T15SECONDS      15000
#define D100METERS      100.0
#define D50METERS       50.0
#define D35METERS       35.0
#define D25METERS       25.0
#define D21METERS       21.0
#define D11METERS       11.0
#define F50FRAMES       50.0

enum STYLELIST { Conservative, Bold } STYLE_ = Bold; // <-- this can be changed
enum STATELIST { Launch, Cruise, Advance, Adapt } STATE_ = Launch;
bool Active = false;
double dist_inc_max = 0.4455;
double steer_limit = STEERINGMAX;

// for convenience
//#define M_PI 3.14159265358979323846
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}
double diff(double ref, double targ) {
  double hbase = MAX_S / 2.0;
  double t = targ - ref;
  return t>hbase ? t - MAX_S : (t<-hbase ? t + MAX_S : t);
}

double triRadius(double a, double b, double c) {
  return (a * b * c) / sqrt((a+b+c)*(b+c-a)*(c+a-b)*(a+b-c));
}

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

double dist_inc = 0.009; // meter distance increment per 20ms frame
Timer1 timerFrame;
Timer1 timerLastLaneChange;
Timer1 timerBlocked;
double LaneTarget[3] = { 2.0, 6.0, 10.0 };
double lastCarS = MAX_S;
long lap_count = 0;
int frameCount = 0;
int curLane = 1; // default lane
int destLane = 1;

// Allocate a spline array
tk::spline laneSpline[3][2] = { {/*lane 0*/},{/*lane 1*/ },{/*lane 2*/ } };

// Persist path state {x,y,s,a,inc} data between update cycles
vector<array<double, 5>> prev_path;

int main() {
  uWS::Hub h;

  // Load map values for waypoint's x,y,s, and normalized d vector
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read
  string map_file_ = "../data/highway_map.csv";
  cout.setf(ios_base::unitbuf);
  cout << "Map file: " << map_file_ << " Driving style is " 
       << (STYLE_?"Bold":"Conservative");
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
  int wpsize = map_waypoints_x.size();

  // Instantiate the lane spline array
  for (int i=0; i<3; i++) {
  vector<double> splinepoints_x; // a set of coordinates down the lane
  vector<double> splinepoints_y;
  vector<double> indexpoints;
  // begin the spline with the last master waypoint...
  splinepoints_x.push_back(map_waypoints_x[wpsize-1] 
    + map_waypoints_dx[wpsize - 1] * LaneTarget[i]);
  splinepoints_y.push_back(map_waypoints_y[wpsize-1] 
    + map_waypoints_dy[wpsize - 1] * LaneTarget[i]);
  indexpoints.push_back(-31.405); // the last waypoint is behind the zero
  for (int j = 0; j < wpsize; j++) { // offset from master waypoints
    splinepoints_x.push_back(map_waypoints_x[j] 
      + map_waypoints_dx[j] * LaneTarget[i]);
    splinepoints_y.push_back(map_waypoints_y[j] 
      + map_waypoints_dy[j] * LaneTarget[i]);
    indexpoints.push_back(map_waypoints_s[j]);
    }
  // and end with the first point for tangential continuity
  splinepoints_x.push_back(map_waypoints_x[0] 
    + map_waypoints_dx[0] * LaneTarget[i]);
  splinepoints_y.push_back(map_waypoints_y[0] 
    + map_waypoints_dy[0] * LaneTarget[i]);
  indexpoints.push_back(MAX_S);

  laneSpline[i][0].set_points(indexpoints, splinepoints_x); // use S as index
  laneSpline[i][1].set_points(indexpoints, splinepoints_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
    &map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, 
    char *data, size_t length, uWS::OpCode opCode) {
  // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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
      //cout << "\r" << car_s << "            ";

      ///////////////////////////////////////////////////////////////////
      // ACCOUNTING
      //

      // Lap counter
      if (lastCarS > 6940.0 && car_s < lastCarS) {
        lap_count++;
        cout << endl << "________________________________________________";
        cout << endl << "Lap " << lap_count;
        cout << endl << "________________________________________________";
      }
      lastCarS = car_s;

      // Check that vehicle is on track
      if (car_d < 0.0 || car_d > 12.0)
        cout << endl << "ROAD EXCEPTION AT: " << car_s;
      assert(car_d >= 0.0 && car_d <= 12.0);
      curLane = car_d < 4.0 ? 0 : (car_d < 8.0 ? 1 : 2); // 0, 1, or 2

      double car_yaw = j[1]["yaw"];
      
      // Velocity check
      double car_speed = j[1]["speed"];
      if (car_speed>=50.0)
        cout << endl << "OVERSPEED EXCEPTION AT: " << car_s;
      assert(car_speed<50.0); // must be less than 50mph

      if (car_speed < 0.5 && frameCount > 200)
        cout << endl << "Underspeed warning at: " << car_s;

      // Previous path data given to the Planner
      auto previous_path_x = j[1]["previous_path_x"];
      int path_size = previous_path_x.size(); // path size from simulator

      if (path_size == 0) { // sim is initialized
        destLane = 1;
        lap_count = 0;
        STATE_ = Launch;
        frameCount = 0;
      }

      // Timers
      Timer1::ms frameDuration = timerFrame.duration(); // frame-to-frame
      long outfd = frameDuration.count();
      if (frameCount++ == 0) {
        timerFrame.restart(); // start a stopwatch
        cout << endl << "t: 0.0 s: " << car_s << " Launching...";
      }

      // Previous path's end s and d values 
      double end_path_s = j[1]["end_path_s"];
      double end_path_d = j[1]["end_path_d"];

      // A list of all other cars on the same side of the road.
      auto sensor_fusion = j[1]["sensor_fusion"];


      ///////////////////////////////////////////////////////////////////
      // SENSOR DATA PREPROCESSING
      //

      // Get closest obstacle distance and velocities
      int num_obstacle = sensor_fusion.size();

      // 2 x 3 array of obstacles 
      //  Ahead: L,C,R   [[ 0, 0, 0 ],
      // Behind: L,C,R    [ 0 ,0, 0 ]]
      double min_dist[2][3] = { { MAXLANEDEPTH,MAXLANEDEPTH,MAXLANEDEPTH },
        { MAXLANEDEPTH,MAXLANEDEPTH,MAXLANEDEPTH } };
      double dist_inc_maxmps = dist_inc_max*F50FRAMES;
      double vel_obj[2][3] = { 
        { dist_inc_maxmps, dist_inc_maxmps, dist_inc_maxmps },
        { dist_inc_maxmps, dist_inc_maxmps, dist_inc_maxmps }
      };

      for (int i = 0; i < num_obstacle; ++i) {
        auto sens = sensor_fusion[i];
        double sens_x = sens[1];
        double sens_y = sens[2];
        double sens_vel_x = sens[3];
        double sens_vel_y = sens[4];
        double sens_s = sens[5];
        double sens_d = sens[6];
        double sens_vel = sqrt(sens_vel_x*sens_vel_x + sens_vel_y*sens_vel_y);
        double dist = diff(car_s, sens_s);
        
        // Handle sensor data anomaly at end of track
        if ((car_s > (MAX_S-100.0) || car_s < 112.0)
          && (sens_s == 0.0) && (sens_d == 0.0)) { 
          // compute distance using dist formula and synthesize d
          dist = distance(sens_x, sens_y, car_x, car_y);
          sens_d = 1136.0 - sens_y; // approx
        }

        double sens_dist = abs(dist);
        int sens_lane = sens_d < 4.0 ? 0 : (sens_d < 8.0 ? 1 : 2); // 0, 1, or 2
        
        // 0 or 1 (0=ahead, 1=behind)  if dist neg, then it is behind
                int sens_pos = dist<0.0;

        // if drifted into curLane, then commit it
        if ( STYLE_ == Conservative
          && ((sens_dist < 20.0 && sens_pos == 0) || 
          (sens_dist < 10.0 && sens_pos == 1))
          && abs(curLane - sens_lane) == 1 // adjacency check
          && abs(car_d - sens_d) < DRIFTTOLERANCE) {
          sens_lane = curLane;
          cout << endl << "Drifting detected " 
            << (sens_pos == 0? "ahead" : "behind")
            << ": " << sens_dist 
            << "m, delta d: " << abs(car_d - sens_d) << "m";

        }

        if (sens_dist < min_dist[sens_pos][sens_lane]) {
          vel_obj[sens_pos][sens_lane] = sens_vel;
          min_dist[sens_pos][sens_lane] = sens_dist;
        }
      }

      // Find the maximum distance in the three lanes ahead
      int prefLane = 1;
      double largerDist = 0;
      for (int i = 0; i < 3; i++) {
        if (min_dist[0][i] > largerDist) {
          prefLane = i;
          largerDist = min_dist[0][i];
        }
      }
      

      ///////////////////////////////////////////////////////////////////
      // BEHAVIOR PLANNER
      //

      double dist_inc_Goal = dist_inc_max;
      double dist_inc_acc = NORMALACCEL;
      double dist_inc_mps = dist_inc*F50FRAMES;
      steer_limit = STEERINGMAX;
      bool replan = false;
      bool doubleLC = false;
      double timeSince = (int(timerFrame.sinceBeginning()/10.0))/100.0;
      // Determine if over the centerline of curLane
      bool on_lane_center = curLane == 0 ? (car_d>1.75 && car_d<2.25) :
        (curLane == 1 ? (car_d>5.75 && car_d<6.25) : (car_d>9.75 && car_d<10.25));

      // FSM to manage modes by checking for transition conditions
      switch (STATE_) {
        case Launch:
          if (dist_inc_mps > VELOCITYLOW) {
            cout << endl << "t: " << timeSince << " s: " << car_s 
              << " Switching to adapt mode";
            STATE_ = Adapt;
          }
          break;
        case Advance: // pushing forward mode
          Active = true;
          dist_inc_acc = 0.001; // easy on the throttle
          dist_inc_Goal = (STYLE_==Conservative ? 1.05 : 1.10) 
            * vel_obj[0][curLane] / F50FRAMES;
          if (dist_inc_Goal > dist_inc_max)
            dist_inc_Goal = dist_inc_max;

          if (min_dist[0][curLane] == MAXLANEDEPTH && on_lane_center
            && dist_inc_mps > VELOCITYHIGH) { // Advance --> Cruise
            cout << endl << "t: " << timeSince << " s: " << car_s 
              << " Switching to cruise mode";
            STATE_ = Cruise;
            Active = false;
          }
          break;

        case Cruise: // cruise mode
          Active = false;
          if (dist_inc_mps < VELOCITYLOW) { // Cruise --> Adapt
            timerBlocked.start();
            cout << endl << "t: " << timeSince << " s: " << car_s 
              << " Adapting to traffic ahead";
            STATE_ = Adapt;
          }
          break;

        case Adapt: // follow the leader...constrained by traffic
          Active = false;
          if (timerBlocked.since().count() > T15SECONDS) {  // Adapt --> Advance
            cout << endl << "t: " << timeSince << " s: " << car_s 
              << " Advancing cautiously";
            STATE_ = Advance;
            Active = true;
          }
          if (dist_inc_mps > VELOCITYLOW && on_lane_center
            && min_dist[0][curLane] >= D100METERS) { // Adapt --> Cruise
            cout << endl << "t: " << timeSince << " s: " << car_s 
              << " Cruising";
            STATE_ = Cruise;
          }
          break;
      }

      // Short-range planning: matching speed or braking
      double brakePercent = 1.0;
      if (min_dist[0][curLane] < (Active ? 30.0 : D50METERS)) {
        brakePercent = // Braking function
          (13.67303 / 0.08660915)* (1 - exp(-0.08660915 * min_dist[0][curLane])) 
          - 55.76378;
        brakePercent = max(0.0, min(100.0, brakePercent));
        brakePercent = brakePercent / 100.0;
        if (brakePercent < 0.8) { // 0.9 is a too high
          replan = true;
          dist_inc_acc = 0.003368; // maximum deceleration
        }
        dist_inc_Goal = brakePercent * vel_obj[0][curLane] / F50FRAMES; // reduce to % of obstacle     50 frames/sec
        if (dist_inc_Goal > dist_inc_max) {
          dist_inc_Goal = dist_inc_max;
          cout << endl << "t: " << timeSince << " s: " << car_s 
            << " Obstacle velocity: " << vel_obj[0][curLane];
        }
      }
      // Attempt to move forward from encroaching vehicles behind
      // by increasing to average of obstacles
      if (STYLE_ == Conservative
        && min_dist[1][curLane] < 10.0 && brakePercent > 0.8) {
        dist_inc_Goal = (vel_obj[0][curLane]+vel_obj[1][curLane]) / 100.0;
        replan = true;
        dist_inc_acc = 0.003368; // maximum acceleration
        if (dist_inc_Goal > dist_inc_max)
          dist_inc_Goal = dist_inc_max;
        cout << endl << "t: " << timeSince << " s: " << car_s 
          << " Push from behind";
      }
      
      // Lane change on fairly straight segments
      bool lcEnabled = false;
      // gather three points on the center lane spline ahead
      double x1 = laneSpline[1][0](car_s);
      double y1 = laneSpline[1][1](car_s);
      double x2 = laneSpline[1][0](car_s + 60.0);
      double y2 = laneSpline[1][1](car_s + 60.0);
      double x3 = laneSpline[1][0](car_s + 120.0);
      double y3 = laneSpline[1][1](car_s + 120.0);
      double a = distance(x1, y1, x2, y2);
      double b = distance(x2, y2, x3, y3);
      double c = distance(x1, y1, x3, y3);
      double r = triRadius(a, b, c);
      if (r > RADIUSSLC) lcEnabled = true;


      // Medium-range planning
      if (lcEnabled && !replan && min_dist[0][curLane] < (Active ? D35METERS : D50METERS+26.0)
        && timerLastLaneChange.since().count() > (Active ?4600:T15SECONDS)) {
        // Lane 1 -> 0 or 2 (whichever is preferred)
        if (curLane != prefLane && prefLane != 1 && curLane == 1
          && min_dist[0][prefLane] > (Active ? D25METERS : D100METERS)
          && min_dist[1][prefLane] > max(D11METERS, 
            3.0*(vel_obj[1][prefLane] - dist_inc_mps) + D11METERS)) {
          destLane = prefLane;
          cout << endl << "t: " << timeSince << " s: " << car_s 
            << " Changing to deeper lane " << prefLane;
          timerLastLaneChange.start();
          replan = true;
          steer_limit = 0.00245;
          if (STATE_ == Active && STYLE_ == Conservative) {
            timerBlocked.start();
            cout << endl << "t: " << timeSince << " s: " << car_s 
              << " Adapting";
            STATE_ = Adapt;
          }
          //dist_inc_Goal = dist_inc_max; // and speed up
        } else

        // Lane 0 -> 1 (also check 2 if Conservative)
        if (curLane == 0 && min_dist[0][1] > (Active ? D25METERS : D50METERS + 26.0)
          && min_dist[1][1] > max(D11METERS, 3.0*(vel_obj[1][1] - dist_inc_mps) + D11METERS)
          && min_dist[0][2] > 13.0 
          && (STYLE_ == Bold || 
          min_dist[1][2] > max(D11METERS, 3.0*(vel_obj[1][2] - dist_inc_mps) + D11METERS) )) {
          destLane = 1;
          cout << endl << "t: " << timeSince << " s: " << car_s 
            << " Changing to lane 1";
          timerLastLaneChange.start();
          replan = true;
          steer_limit = 0.00245;
          if (STATE_ == Active && STYLE_ == Conservative) {
            timerBlocked.start();
            cout << endl << "t: " << timeSince << " s: " << car_s 
              << " Adapting";
            STATE_ = Adapt;
          }
          //dist_inc_Goal = dist_inc_max;
        } else

        // Lane 2 -> 1 (also check 0 if Conservative)
        if (curLane == 2 && min_dist[0][1] > (Active ? D25METERS : D50METERS + 26.0)
          && min_dist[1][1] > max(D11METERS, 3.0*(vel_obj[1][1] - dist_inc_mps) + D11METERS)
          && min_dist[0][0] > 13.0 
          && ( STYLE_ == Bold || 
            min_dist[1][0] > max(D11METERS, 3.0*(vel_obj[1][0] - dist_inc_mps) + D11METERS) )) {
          destLane = 1;
          cout << endl << "t: " << timeSince << " s: " << car_s 
            << " Changing to lane 1";
          timerLastLaneChange.start();
          replan = true;
          steer_limit = 0.00245;
          if (STATE_ == Active && STYLE_ == Conservative) {
            timerBlocked.start();
            cout << endl << "t: " << timeSince << " s: " << car_s 
              << " Adapting";
            STATE_ = Adapt;
          }
          //dist_inc_Goal = dist_inc_max;
        } else

        // Lane 0 -> 2 (must check 1) DOUBLE LC on straight sections
        if (r > RADIUSDLC
          && curLane == 0 && min_dist[0][1] > (Active ? 20.0 : 45.0)
          && min_dist[1][1] > max(D11METERS, 3.0*(vel_obj[1][1] - dist_inc_mps) + D11METERS)
          && min_dist[0][2] > D50METERS && min_dist[1][2] > D50METERS) {
          destLane = 2;
          cout << endl << "t: " << timeSince << " s: " << car_s 
            << " Going around to lane 2" << " r: " << r;
          timerLastLaneChange.start();
          replan = true;
          doubleLC = true;
          steer_limit = 0.00245;
          if (STATE_ == Active && STYLE_ == Conservative) {
            timerBlocked.start();
            cout << endl << "t: " << timeSince << " s: " << car_s 
              << " Adapting";
            STATE_ = Adapt;
          }
          //dist_inc_Goal = dist_inc_max;
        } else

        // Lane 2 -> 0 (must check 1) DOUBLE LC
        if (r > RADIUSDLC
          && curLane == 2 && min_dist[0][1] > (Active ? 20.0 : 45.0)
          && min_dist[1][1] > max(D11METERS, 3.0*(vel_obj[1][1] - dist_inc_mps) + D11METERS)
          && min_dist[0][0] > D50METERS && min_dist[1][0] > D50METERS) {
          destLane = 0;
          cout << endl << "t: " << timeSince << " s: " << car_s 
            << " Going around to lane 0" << " r: " << r;
          timerLastLaneChange.start();
          replan = true;
          doubleLC = true;
          steer_limit = 0.00245;
          if (STATE_ == Active && STYLE_ == Conservative) {
            timerBlocked.start();
            cout << endl << "t: " << timeSince << " s: " << car_s 
              << " Adapting";
            STATE_ = Adapt;
          }
          //dist_inc_Goal = dist_inc_max;
        }
      }

      // Long-range planning: if lanes are clear and timer has expired, return to lane 1 
      if (lcEnabled && !replan && min_dist[0][curLane] > (Active ? D35METERS : D100METERS)
        && timerLastLaneChange.since().count() > (Active ? T5SECONDS : T15SECONDS)) {
        
        // Lane 0 or 2 -> 1 preference to stay in middle (check far lane)
        if (((curLane == 2 && min_dist[0][1] == MAXLANEDEPTH && min_dist[1][1] > D21METERS
          && min_dist[0][0] > D50METERS && min_dist[1][0] > D50METERS)   || 
          (curLane == 0 && min_dist[0][1] == MAXLANEDEPTH && min_dist[1][1] > D21METERS
          && min_dist[0][2] > D50METERS && min_dist[1][2] > D50METERS)) ) {
            destLane = 1;
            cout << endl << "t: " << timeSince << " s: " << car_s 
              << " Returning to lane 1";
            timerLastLaneChange.start();
            steer_limit = 0.00245;
        } else

        // Lane 1 -> 0 early lane change around upcoming traffic
        if ( ((curLane == 1 && min_dist[0][0] == MAXLANEDEPTH && min_dist[1][0] > D21METERS) 
          && min_dist[0][1] < (Active ? 50 : 90)) ) {
            destLane = 0;
            cout << endl << "t: " << timeSince << " s: " << car_s 
              << " Early switch to lane 0";
            timerLastLaneChange.start();
            steer_limit = 0.00245;
        } else

        // Lane 1 -> 2 early lane change around upcoming traffic
        if (((curLane == 1 && min_dist[0][2] == MAXLANEDEPTH && min_dist[1][2] > D21METERS)
          && min_dist[0][1] < (Active ? 50 : 90))) {
            destLane = 2;
            cout << endl << "t: " << timeSince << " s: " << car_s 
              << " Early switch to lane 2";
            timerLastLaneChange.start();
            steer_limit = 0.00245;
        }
      }

      // TRACK REPAIR (overspeed issue)
      if (car_s <= 3200.0 && car_s >= 2900.0 && dist_inc_Goal == dist_inc_max)
        dist_inc_Goal = 0.445; // slow down in the northeast corner


      ///////////////////////////////////////////////////////////////////
      // MOTION CONTROLLER
      //

      vector<double> next_x_vals;
      vector<double> next_y_vals;

      double pos_x;
      double pos_y;
      double pos_s;
      double angle = 0.0;

      if (path_size == 0) { // sim is initialized
        pos_x = car_x;
        pos_y = car_y;
        pos_s = car_s;
        angle = deg2rad(car_yaw);
        dist_inc = 0.009;
      }
      else {
        assert(path_size>1); // must be 2 or more items on the path

        // remove expired items from the previous path
        int prevlength = prev_path.size();
        prev_path.erase(prev_path.begin(), prev_path.begin()+(prevlength-path_size));
        assert(prev_path.size() == path_size);  // double check erasure

        if (replan) {
          path_size = 4;
        }

        for (int i = 0; i < path_size; i++) { // reuse previous path
          next_x_vals.push_back(prev_path[i][0]);
          next_y_vals.push_back(prev_path[i][1]);
        }

        if (replan) {
          prev_path.erase(prev_path.begin()+path_size, prev_path.end());
          assert(prev_path.size() == path_size);  // double check erasure
          dist_inc = prev_path[path_size - 1][4];
          assert(dist_inc <= dist_inc_max);
          end_path_s = prev_path[path_size - 1][2];
        }
        pos_x = prev_path[path_size-1][0];
        pos_y = prev_path[path_size-1][1];
        pos_s = end_path_s;
        angle = prev_path[path_size-1][3];
      }

      int frameCnt = QDEPTH - path_size;
      
      for (int i = 0; i < frameCnt; i++) {
        double refAngle = 0.0, steering = 0.0;
        double wpoffsetx = 0.0, wpoffsety = 0.0;

        pos_s += dist_inc;
        if (pos_s > MAX_S) pos_s -= MAX_S; // modulo

        // target a point several meters ahead in the current lane
        double refTarget = pos_s + TARGETPOINT;
        if (refTarget > MAX_S) refTarget -= MAX_S; // modulo

        wpoffsetx = laneSpline[destLane][0](refTarget);
        wpoffsety = laneSpline[destLane][1](refTarget);

        // transform lane waypoint to vehicle coordinates
        double dx = wpoffsetx - pos_x; // translate
        double dy = wpoffsety - pos_y;

        double cosPsi = cos(-angle);
        double sinPsi = sin(-angle);
        wpoffsetx = dx * cosPsi - dy * sinPsi; // rotate
        wpoffsety = dy * cosPsi + dx * sinPsi;

        // find bearing to waypoint
        refAngle = atan2(wpoffsety, wpoffsetx);

        steering = refAngle;
        if (refAngle > 0) { // steer left
          if (steering > steer_limit)
            steering = steer_limit;
        }
        if (refAngle < 0) { // steer right
          if (steering < -steer_limit)
            steering = -steer_limit;
        }

        // launching and stopping
        if (STATE_ == Launch) {
          steering = 0.0;
          dist_inc_acc = 0.0039; // 00394 causes maxacc err on slow Mac
        } else if (dist_inc < 0.05) {
          steering = 0.0; // too slow, don't steer
        }

        angle += steering;
        double last_x = pos_x;
        double last_y = pos_y;
        pos_x += (dist_inc)*cos(angle);
        pos_y += (dist_inc)*sin(angle);

        // accelerating
        if (dist_inc < dist_inc_Goal) {
          dist_inc += dist_inc_acc;
          if (dist_inc > dist_inc_Goal) dist_inc = dist_inc_Goal;
        }
        // decelerating
        if (dist_inc > dist_inc_Goal) {
          dist_inc -= dist_inc_acc;
          if (dist_inc < dist_inc_Goal) dist_inc = dist_inc_Goal;
        }

        // push trajectory data to the simulator
        next_x_vals.push_back(pos_x);
        next_y_vals.push_back(pos_y);
        // ...and save it for next cycle
        prev_path.push_back({ pos_x,pos_y,pos_s,angle,dist_inc });
      }
      
      json msgJson;
      msgJson["next_x"] = next_x_vals;
      msgJson["next_y"] = next_y_vals;

            auto msg = "42[\"control\","+ msgJson.dump()+"]";

            // OPTIONAL: simulate latency
            // this_thread::sleep_for(chrono::milliseconds(150));
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
  // program doesn't compile :-(
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
    std::cout << endl << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << endl << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << std::endl << "Listening to port " << port;
  } else {
    std::cerr << std::endl << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}