#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include <chrono>
#include <thread>
#include "spline.h"

#include <math.h>
#include <string>
#include <vector>

#ifndef HELPERS_h
#define HELPERS_h



// for convenience
using std::string;
using std::vector;
using namespace std;
using nlohmann::json;


//=== Starting lane in the middle
int lane = 1;

//=== Initial vel (mph)
double vel0 = 0.0;

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
    
    ifstream in_map_(map_file_.c_str(), std::ifstream::in);
    
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
    
    
    
    h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
                 &map_waypoints_dx,&map_waypoints_dy]
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
                                
                                //=== Number of points in previous path
                                int histSize = previous_path_x.size();
                                
                                //=== Check for probability to have a collision
                                bool colide = false;
                                
                                if (histSize > 0) {
                                    colide = true;
                                }
                                //=== Preventing the collision
                                if (colide){
                                    car_s = end_path_s;
                                }
                                
                                //=== where are other cars?
                                bool obst_front = false;
                                bool obst_left = false;
                                bool obst_right = false;
                                for ( int i = 0; i < sensor_fusion.size(); i++ ) {
                                    //=== location of the obstacle from the sensors
                                    float d = sensor_fusion[i][6];
                                    int obst_lane = -1;
                                    // is it on the same lane we are
                                    if ( d > 0 && d < 4 ) {
                                        obst_lane = 0;
                                    } else if ( d > 4 && d < 8 ) {
                                        obst_lane = 1;
                                    } else if ( d > 8 && d < 12 ) {
                                        obst_lane = 2;
                                    }
                                    if (obst_lane < 0) {
                                        continue;
                                    }
                                    
                                    //=== velocity
                                    double vel_lon = sensor_fusion[i][3]; // Longitudinal velocity
                                    double vel_lat = sensor_fusion[i][4]; // Lateral velocity
                                    
                                    //=== Magnitude of velocity in motion
                                    double vel_mag = sqrt(vel_lon*vel_lon + vel_lat*vel_lat);
                                    
                                    //=== Psition of the car based on having constant velocity from
                                    //=== previous measurment
                                    double check_car_s = sensor_fusion[i][5];
                                    //=== Estimate car pred position after executing previous trajectory.
                                    check_car_s += ((double)histSize*0.02*vel_mag);
                                    
                                    if ( obst_lane == lane ) {
                                        //=== Obstacle is in the lane
                                        obst_front |= check_car_s > car_s && check_car_s - car_s < 30;
                                    } else if ( obst_lane - lane == -1 ) {
                                        //=== Obstacle is on the left
                                        obst_left |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;
                                    } else if ( obst_lane - lane == 1 ) {
                                        //=== Obstacle is on the right
                                        obst_right |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;
                                    }
                                }
                                
                                //=== re-actions
                                double speed_diff = 0;
                                const double MAX_SPEED = 49.5;
                                const double MAX_ACC = .224;
                                if ( obst_front ) { // Car ahead
                                    if ( !obst_left && lane > 0 ) {
                                        lane--; //=== Moving to the left.
                                    } else if ( !obst_right && lane != 2 ){
                                        // if there is no car right and there is a right lane.
                                        lane++; //=== Moving to the right.
                                    } else {
                                        speed_diff -= MAX_ACC;
                                    }
                                } else {
                                    // Ceck the position in lanes
                                    if ( lane != 1 ) {
                                        // Correcting
                                        if ( ( lane == 0 && !obst_right ) || ( lane == 2 && !obst_left ) ) {
                                            lane = 1;
                                        }
                                    }
                                    if ( vel0 < MAX_SPEED ) {
                                        speed_diff += MAX_ACC;
                                    }
                                }
                                
                                vector<double> pt_lat; // Longitudinal motion
                                vector<double> pt_lon; // Lateral motion
                                
                                double lon_current = car_x; // Longitudinal position
                                double lat_current = car_y; // Lateral position
                                double ref_yaw = deg2rad(car_yaw);
                                
                                //=== previous pts
                                if ( histSize < 2 ) {
                                    // There are not too many...
                                    double prev_car_x = car_x - cos(car_yaw);
                                    double prev_car_y = car_y - sin(car_yaw);
                                    
                                    pt_lat.push_back(prev_car_x); // Begining of motion in lon
                                    pt_lat.push_back(car_x);      // End of motion in lon
                                    
                                    pt_lon.push_back(prev_car_y); // Begining of motion in lat
                                    pt_lon.push_back(car_y);     // End of motion in lon
                                } else {
                                    // Use the last two points.
                                    lon_current = previous_path_x[histSize - 1];
                                    lat_current = previous_path_y[histSize - 1];
                                    
                                    double ref_x_prev = previous_path_x[histSize - 2];
                                    double ref_y_prev = previous_path_y[histSize - 2];
                                    ref_yaw = atan2(lat_current-ref_y_prev, lon_current-ref_x_prev);
                                    
                                    pt_lat.push_back(ref_x_prev);
                                    pt_lat.push_back(lon_current);
                                    
                                    pt_lon.push_back(ref_y_prev);
                                    pt_lon.push_back(lat_current);
                                }
                                
                                //=== Next dsetination by using 3 fernet pts by the dostance of 30 m from each other (30, 60, 90 m)
                                vector<double> wp_30 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                                vector<double> wp_60 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                                vector<double> wp_90 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                                
                                pt_lat.push_back(wp_30[0]);
                                pt_lat.push_back(wp_60[0]);
                                pt_lat.push_back(wp_90[0]);
                                
                                pt_lon.push_back(wp_30[1]);
                                pt_lon.push_back(wp_60[1]);
                                pt_lon.push_back(wp_90[1]);
                                
                                //=== defining local coords by puttin gthe car at the instantanous center of the motion
                                for ( int i = 0; i < pt_lat.size(); i++ ) {
                                    double shift_x = pt_lat[i] - lon_current;
                                    double shift_y = pt_lon[i] - lat_current;
                                    
                                    pt_lat[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
                                    pt_lon[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
                                }
                                
                                //=== fitting a spline
                                tk::spline pred;
                                pred.set_points(pt_lat, pt_lon);
                                
                                //=== Output path points from previous path for continuity.
                                vector<double> next_x_vals;
                                vector<double> next_y_vals;
                                for ( int i = 0; i < histSize; i++ ) {
                                    next_x_vals.push_back(previous_path_x[i]);
                                    next_y_vals.push_back(previous_path_y[i]);
                                }
                                
                                //=== Calculate distance y position on `target_x` m ahead.
                                double target_x = 30.0;
                                double target_y = pred(target_x);
                                double target_dist = sqrt(target_x*target_x + target_y*target_y);
                                
                                double x_add_on = 0;
                                
                                for( int i = 1; i < 50 - histSize; i++ ) {
                                    vel0 += speed_diff;
                                    if ( vel0 > MAX_SPEED ) {
                                        vel0 = MAX_SPEED;
                                    } else if ( vel0 < MAX_ACC ) {
                                        vel0 = MAX_ACC;
                                    }
                                    double N = target_dist/(0.02*vel0/2.24);
                                    double x_point = x_add_on + target_x/N;
                                    double y_point = pred(x_point);
                                    
                                    x_add_on = x_point;
                                    
                                    double x_ref = x_point;
                                    double y_ref = y_point;
                                    
                                    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
                                    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
                                    
                                    x_point += lon_current;
                                    y_point += lat_current;
                                    
                                    next_x_vals.push_back(x_point);
                                    next_y_vals.push_back(y_point);
                                }
                                
                                json msgJson;
                                
                                msgJson["next_x"] = next_x_vals;
                                msgJson["next_y"] = next_y_vals;
                                
                                auto msg = "42[\"control\","+ msgJson.dump()+"]";
                                
                                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                                
                            }
                        } else {
                            // Manual driving
                            std::string msg = "42[\"manual\",{}]";
                            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                        }
                    }
                });
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
#endif /* HELPERS_h */

