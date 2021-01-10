#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


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
  int lane = 1;
  double ref_vel = 0;

  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
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
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

           bool in_reach = false;
           bool left_detection = false;
           bool right_detection = false;

           if(previous_path_x.size() > 0)
           {
             car_s = end_path_s;
           }

           for(int i = 0; i < sensor_fusion.size(); i++)
           {
             double vel_x = sensor_fusion[i][3];
             double vel_y = sensor_fusion[i][4];
             double speed_checker = sqrt(vel_x*vel_x + vel_y*vel_y);
             double check_car_s = sensor_fusion[i][5];
             float distance = sensor_fusion[i][6];
             int current_lane = -1;
             if(0 < distance && distance < 4)
             {
               current_lane = 0;
             } else if(4 < distance && distance < 8)
             {
               current_lane = 1;
             } else if(8 < distance && distance < 12)
             {
               current_lane = 2;
             } else {
               continue;
             }
             int check_buffer = 30;
             check_car_s +=((double)speed_checker * previous_path_x.size() * 0.02);
             if ( current_lane == lane )
             {
               in_reach = in_reach | (check_car_s > car_s && check_car_s - car_s < check_buffer);
             } else if ( current_lane + 1 == lane )
             {
               left_detection = left_detection | (car_s - check_buffer < check_car_s && car_s + check_buffer > check_car_s);
             } else if ( current_lane - 1 == lane )
             {
               right_detection = right_detection | (car_s - check_buffer < check_car_s && car_s + check_buffer > check_car_s);
              }
           }

          double vel_diff = 0;
          if(in_reach)
          {
            if(lane > 0 && !left_detection)
            {
              lane--;
            }else if(lane < 2 && !right_detection)
            {
              lane++;
            }else
            {
              ref_vel -= 0.224;
            }
          }else
          {
            if(lane != 1)
            {
              if(( lane == 0 && !right_detection ) || ( lane == 2 && !left_detection ) )
              {
                lane = 1;
              }
            }
            if(ref_vel < 49.5)
            {
              ref_vel += 0.224;
            }
          }


          double ref_raw = deg2rad(car_yaw);
          vector<double> x_points;
          vector<double> y_points;
          double car_ref_x = car_x;
          double car_ref_y = car_y;
          if(previous_path_x.size() > 1)
          {
            double prev_cur_x_ref =  previous_path_x[previous_path_x.size() - 2];
            double prev_cur_y_ref =  previous_path_y[previous_path_x.size() - 2];
            car_ref_x = previous_path_x[previous_path_x.size() - 1];
            car_ref_y = previous_path_y[previous_path_x.size() - 1];
            ref_raw = atan2(car_ref_y - prev_cur_y_ref, car_ref_x - prev_cur_x_ref);
            x_points.push_back(prev_cur_x_ref);
            x_points.push_back(car_ref_x);
            y_points.push_back(prev_cur_y_ref);
            y_points.push_back(car_ref_y);
          }else
          {
            double car_x_prev = car_x - cos(car_yaw);
            double car_y_prev = car_y - sin(car_yaw);
            x_points.push_back(car_x_prev);
            x_points.push_back(car_x);
            y_points.push_back(car_y_prev);
            y_points.push_back(car_y);
          }
          vector<double> wp0_next = getXY(30 + car_s, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> wp1_next = getXY(60 + car_s, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> wp2_next = getXY(90 + car_s, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          x_points.push_back(wp0_next[0]);
          x_points.push_back(wp1_next[0]);
          x_points.push_back(wp2_next[0]);
          y_points.push_back(wp0_next[1]);
          y_points.push_back(wp1_next[1]);
          y_points.push_back(wp2_next[1]);
          for(int i = 0; i < x_points.size(); ++i)
          {
            double x_change = x_points[i] - car_ref_x;
            double y_change = y_points[i] - car_ref_y;
            x_points[i] = x_change * cos(0 - ref_raw) - y_change * sin(0 - ref_raw);
            y_points[i] = x_change * sin(0 - ref_raw) + y_change * cos(0 - ref_raw);
          }

          tk::spline _spline;
          _spline.set_points(x_points, y_points);
          for(int i = 0; i < previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double dist_x = 30.0;
          double dist_y = _spline(dist_x);
          double dist_target = sqrt(dist_x * dist_x + dist_y * dist_y);
          double total_x = 0;
          for(int i = 0; i <= 50 - previous_path_x.size(); ++i)
          {
            double N = dist_target / (0.02 * ref_vel/2.24);
            double x_point = total_x + dist_x / N;
            double y_point = _spline(x_point);
            total_x = x_point;
            double cur_x_ref = x_point;
            double cur_y_ref = y_point;
            x_point = cur_x_ref * cos(ref_raw) - cur_y_ref * sin(ref_raw);
            y_point = cur_x_ref * sin(ref_raw) + cur_y_ref * cos(ref_raw);
            x_point += car_ref_x;
            y_point += car_ref_y;
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
