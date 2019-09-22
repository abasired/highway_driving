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
#include <math.h>
#include <cmath>
using namespace std;

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
  int lane = 1;
  double ref_vel = 0;
  double target_vel = 25;
 
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
  
  
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel, &target_vel]
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
         
          // Length of previous path still to be covered
          int prev_size = previous_path_x.size();
          
          int Num_highway_lanes = 3;
          int LANE_WIDTH = 4;
          int current_car_lane;
          float d;
          double vx;
          double vy;
          double check_speed;
          double check_car_s;
          double vel_ratio;
          double s_diff;
          
          bool too_close = false;
          bool lane_change_right_gap = true;
          bool lane_change_left_gap = true;
          
           
          if(prev_size > 0)
          {
            car_s = end_path_s;
          }
         
          std::ofstream myfile("lane_info.txt", std::ios_base::app | std::ios_base::out);
          myfile << "car values" << "\t" << car_s << "\t"<< ref_vel/2.24 << "\t"<< car_d << "\t"<< "current_lane" <<"\t"<< lane << "\t"<< vel_ratio <<"\t" << prev_size <<"\n";
          /*
          Pericive the surrounding with respect to every car in sensor fusion output and compute the cost for every action.
          */
          for(int i = 0; i < sensor_fusion.size(); i++)
          {
            d = sensor_fusion[i][6];
            vx = sensor_fusion[i][3];
            vy = sensor_fusion[i][4];
            check_speed = sqrt(vx*vx +vy*vy);
            check_car_s = sensor_fusion[i][5];
            check_car_s += ((double)prev_size*0.02*check_speed);
            vel_ratio = (ref_vel/2.24)/check_speed;
            s_diff = car_s - check_car_s;
            //int ngbr_lane = d % 4;
            
            for (int i = 0; i < Num_highway_lanes; i++) {
              if (d > i * LANE_WIDTH && d < (i + 1) * LANE_WIDTH) {
                current_car_lane = i;
                break;
              }
            }

            if(lane == current_car_lane) {
                too_close = too_close || (check_car_s > car_s && check_car_s - car_s < 30);
                //cost_KL.push_back(((target_vel - ref_vel) + (target_vel - check_speed))/target_vel);
                }
              myfile << "ngbr lane car values" << "\t"<< check_car_s << "\t"<< check_speed << "\t"<< d <<"\t" <<  "too_close" <<"\t"<< too_close <<"\t" << s_diff <<"\n";

              
            if (current_car_lane == lane - 1 ){
                lane_change_left_gap = lane_change_left_gap && pow(vel_ratio, s_diff) > 1 && abs(s_diff) > 30 ;
                //cost_LCL.push_back(((target_vel - ref_vel) + (target_vel - check_speed))/target_vel);
                } 
              
               myfile << "ngbr lane car values" << "\t"<< check_car_s << "\t"<< check_speed << "\t"<< d <<"\t" <<  "lane_change_left_gap" <<"\t"<< lane_change_left_gap <<"\t" << s_diff <<"\n";
             
             
            if (current_car_lane == lane + 1){
              lane_change_right_gap = lane_change_right_gap && pow(vel_ratio, s_diff) > 1 && abs(s_diff) > 30;
              }
              
              myfile << "ngbr lane car values" << "\t"<< check_car_s << "\t"<< check_speed << "\t"<< d <<"\t" <<  "lane_change_right_gap" <<"\t"<< lane_change_right_gap <<"\t" << s_diff <<"\n";
         
          }
          
          /*
          If there is slow moving vehicle in the lane try to change lane. Initially perform lane change check. 
          Then set the flag for respective lange change. Udate the lane information for lane change trajectory generation.
          */
            if(too_close)
            {
              // make decision based on the flags set.
              if((lane_change_left_gap) && lane > 0 ){
                lane = lane - 1;              
              }else if(lane_change_right_gap && lane < 2 ){
                lane = lane + 1;
              }else if (ref_vel > check_speed){
                ref_vel -= 0.224;
              }else {
                ref_vel = check_speed;
              }
            }
            else if (ref_vel < 49.50 - 3*0.224)
            { 
              ref_vel += 3*0.224;
            }

               myfile.close();            
 
          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          // points and few from prevopus path that are located at s=30,60,90 . Used for spline interpolation.
                 
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          vector<double> ptsx;
          vector<double> ptsy;
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          if(prev_size < 2)
          {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
           
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else 
          {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
                    
          // Setting up target points in the future.
            vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

          
               
          // points in local coordinates of car.
          for (int i=0; i < ptsx.size();i++)
          {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
            ptsy[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
          }
          
          /*
          std::ofstream myfile("data_info.txt", std::ios_base::app | std::ios_base::out);
          myfile << "values of ptsx" << "\t"<< ptsx[0] << "\t"<< ptsx[1] << "\t"<< ptsx[2] << "\t"<<  ptsx[3] << "\t"<<  ptsx[4]<< "\n";
          myfile << "car values" << "\t" << car_x << "\t"<< car_y << "\t"<< car_yaw << "\t"<< car_s << "\t"<< car_d <<"\n";
          myfile.close();
          */
          tk::spline s;
          s.set_points(ptsx,ptsy);
          
          
          // Generating a new path using previous path and rest from spline interpolated path.
          
          for(int i = 0; i< previous_path_x.size(); i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          /**
          Rest of the points are generated from spline interpolated trajectory.
          These are covereted back into map coordinates          
          **/
          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y*target_y);
          double N= target_dist/(0.02*(ref_vel/2.24));
          double x_add_on = 0;
          
          for (int i = 1; i < 50 - previous_path_x.size(); i++) {
            
            double x_point = x_add_on + (target_x) / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;
            
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