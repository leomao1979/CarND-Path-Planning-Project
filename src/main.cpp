#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <limits>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;
static const double SPEED_LIMIT = 49.5 / 2.24;      // mps (49.5 mph)
static const double DEFAULT_BUFFER_ZONE = 25;       // default buffer
static const double MINIMUM_BUFFER_ZONE = 10;       // minimum buffer
static const int    TOTAL_LANES = 3;                // total lanes

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
    
    double heading = atan2((map_y-y),(map_x-x));
    
    double angle = fabs(theta-heading);
    angle = min(2*pi() - angle, angle);
    
    if(angle > pi()/4)
    {
        closestWaypoint++;
        if (closestWaypoint == maps_x.size())
        {
            closestWaypoint = 0;
        }
    }
    
    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
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

// Returns "velocity" and "s" of vehicle ahead. If not found, returns empty vector
vector<double> getVehicleAhead(const vector<vector<double>>& sensor_fusion, int lane, double s, double pred_time) {
    vector<double> result;
    double min_s = std::numeric_limits<double>::max();
    for (int i=0; i<sensor_fusion.size(); i++) {
        double i_d = sensor_fusion[i][6];
        if (i_d >= lane * 4 && i_d < (lane + 1) * 4) {
            // same lane
            double i_vx = sensor_fusion[i][3];
            double i_vy = sensor_fusion[i][4];
            double check_v = sqrt(i_vx*i_vx + i_vy*i_vy);
            double i_s = sensor_fusion[i][5];
            double check_s = i_s + pred_time * check_v;
            if (check_s > s && check_s < min_s) {
                min_s = check_s;
                result.clear();
                result.push_back(check_v);
                result.push_back(check_s);
            }
        }
    }
    return result;
}

// Returns "velocity" and "s" of vehicle behind. If not found, returns empty vector
vector<double> getVehicleBehind(const vector<vector<double>>& sensor_fusion, int lane, double s, double pred_time) {
    vector<double> result;
    double max_s = 0;
    for (int i=0; i<sensor_fusion.size(); i++) {
        double i_d = sensor_fusion[i][6];
        if (i_d >= lane * 4 && i_d < (lane + 1) * 4) {
            // same lane
            double i_vx = sensor_fusion[i][3];
            double i_vy = sensor_fusion[i][4];
            double check_v = sqrt(i_vx*i_vx + i_vy*i_vy);
            double i_s = sensor_fusion[i][5];
            double check_s = i_s + pred_time * check_v;
            if (check_s < s && check_s > max_s) {
                max_s = check_s;
                result.clear();
                result.push_back(check_v);
                result.push_back(check_s);
            }
        }
    }
    return result;
}

double calculateLaneBuffer(const vector<vector<double>>& sensor_fusion, int lane, double s, double pred_time) {
    double lane_buffer = -1;
    vector<double> vehicle_behind = getVehicleBehind(sensor_fusion, lane, s, pred_time);
    double vehicle_behind_s = std::numeric_limits<double>::min();
    if (vehicle_behind.size() > 0) {
        vehicle_behind_s = vehicle_behind[1];
    }
    if (s - vehicle_behind_s > MINIMUM_BUFFER_ZONE) {
        vector<double> vehicle_ahead = getVehicleAhead(sensor_fusion, lane, s, pred_time);
        double vehicle_ahead_s = std::numeric_limits<double>::max();
        if (vehicle_ahead.size() > 0) {
            vehicle_ahead_s = vehicle_ahead[1];
        }
        lane_buffer = vehicle_ahead_s - s;
    }
    return lane_buffer;
}

int findLaneToChange(const vector<vector<double>>& sensor_fusion, int lane, double s, double pred_time) {
    int change_to_lane = -1;
    double left_lane_buffer = -1;
    double right_lane_buffer = -1;
    if (lane > 0) {
        left_lane_buffer = calculateLaneBuffer(sensor_fusion, lane - 1, s, pred_time);
    }
    if (lane < TOTAL_LANES - 1) {
        right_lane_buffer = calculateLaneBuffer(sensor_fusion, lane + 1, s, pred_time);
    }
    if (left_lane_buffer > DEFAULT_BUFFER_ZONE) {
        change_to_lane = lane - 1;
    }
    if (right_lane_buffer > left_lane_buffer && right_lane_buffer > DEFAULT_BUFFER_ZONE) {
        change_to_lane = lane + 1;
    }
    return change_to_lane;
}

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
    double ref_velocity = 0.0;                  // mps (meters per second)
    double default_acceleration = 0.18;         // velocity change (mps) in 20ms. acceleration: 0.18 * 50 = 9m/s^2
    int    next_lane = -1;
    
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
    
    h.onMessage([&ref_velocity, &default_acceleration, &next_lane, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
                    double car_speed = j[1]["speed"];       // mps
                    int    car_lane = car_d / 4;
                    
                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    int  prev_size = previous_path_x.size();
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];
                    if (prev_size == 0) {
                        end_path_s = car_s;
                        end_path_d = car_d;
                    }

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    if (next_lane == -1) {
                        next_lane = car_lane;
                    }
                    // Calculate velocity and next lane
                    double pred_time = prev_size * 0.02;
                    vector<double> vehicle_ahead = getVehicleAhead(sensor_fusion, next_lane, end_path_s, pred_time);
                    if (car_lane != next_lane) {
                        // Changing lane. Slow down if it is too close with the vehicle ahead in either current lane, or change-to lane
                        bool shall_slowdown = false;
                        if (vehicle_ahead.size() > 0) {
                            double v = vehicle_ahead[0];
                            double s = vehicle_ahead[1];
                            if (s - end_path_s <= MINIMUM_BUFFER_ZONE && car_speed > v) {
                                shall_slowdown = true;
                            }
                        }
                        if (!shall_slowdown) {
                            vector<double> samelane_vehicle_ahead = getVehicleAhead(sensor_fusion, car_lane, end_path_s, pred_time);
                            if (samelane_vehicle_ahead[1] - end_path_s <= MINIMUM_BUFFER_ZONE && car_speed > samelane_vehicle_ahead[0]) {
                                shall_slowdown = true;
                            }
                        }
                        if (shall_slowdown) {
                            ref_velocity -= default_acceleration;
                        }
                        cout << "Changing lane from lane #" << car_lane << " to lane #" << next_lane << ". ref_velocity: " << ref_velocity << endl;
                    } else {
                        if (vehicle_ahead.size() == 0) {
                            // No vehicle ahead
                            ref_velocity += default_acceleration;
                        } else {
                            double v = vehicle_ahead[0];
                            double s = vehicle_ahead[1];
                            if (s - end_path_s > DEFAULT_BUFFER_ZONE) {
                                ref_velocity += default_acceleration;
                            } else if (car_speed > v) {
                                int lane_to_change = findLaneToChange(sensor_fusion, car_lane, end_path_s, pred_time);
                                if (lane_to_change != -1) {
                                    next_lane = lane_to_change;
                                    cout << "car_lane: " << car_lane << "; next_lane: " << next_lane << endl;
                                } else {
                                    ref_velocity -= default_acceleration;
                                }
                            }
                        }
                    }

                    if (ref_velocity > SPEED_LIMIT) {
                        ref_velocity = SPEED_LIMIT;
                    } else if (ref_velocity < 0) {
                        ref_velocity = 0;
                    }
                    if (ref_velocity > 0.001) {
                        vector<double> ptsx, ptsy;
                        double ref_x = car_x;
                        double ref_y = car_y;
                        double ref_yaw = deg2rad(car_yaw);
                        if (prev_size < 2) {
                            double prev_car_x = car_x - cos(ref_yaw);
                            double prev_car_y = car_y - sin(ref_yaw);
                            ptsx.push_back(prev_car_x);
                            ptsx.push_back(car_x);
                            ptsy.push_back(prev_car_y);
                            ptsy.push_back(car_y);
                        } else {
                            ref_x = previous_path_x[prev_size-1];
                            ref_y = previous_path_y[prev_size-1];
                            double ref_x_prev = previous_path_x[prev_size-2];
                            double ref_y_prev = previous_path_y[prev_size-2];
                            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
                            ptsx.push_back(ref_x_prev);
                            ptsx.push_back(ref_x);
                            ptsy.push_back(ref_y_prev);
                            ptsy.push_back(ref_y);
                        }
                        vector<double> next_wp0 = getXY(end_path_s+30, next_lane*4 + 2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                        vector<double> next_wp1 = getXY(end_path_s+60, next_lane*4 + 2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                        vector<double> next_wp2 = getXY(end_path_s+90, next_lane*4 + 2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                        ptsx.push_back(next_wp0[0]);
                        ptsx.push_back(next_wp1[0]);
                        ptsx.push_back(next_wp2[0]);
                        ptsy.push_back(next_wp0[1]);
                        ptsy.push_back(next_wp1[1]);
                        ptsy.push_back(next_wp2[1]);
                        
                        // Transform to local coordinate system
                        for (int i=0; i<ptsx.size(); i++) {
                            double shift_x = ptsx[i] - ref_x;
                            double shift_y = ptsy[i] - ref_y;
                            ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
                            ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
                        }
                        tk::spline s;
                        s.set_points(ptsx, ptsy);
                        
                        for(int i = 0; i < prev_size; i++) {
                            next_x_vals.push_back(previous_path_x[i]);
                            next_y_vals.push_back(previous_path_y[i]);
                        }
                        
                        double target_x = 30.0;
                        double target_y = s(target_x);
                        double target_dist = sqrt(target_x * target_x + target_y * target_y);
                        double N = target_dist / (0.02 * ref_velocity);
                        double step_len = target_dist / N;
                        for(int i = 1; i <= 50 - prev_size; i++)
                        {
                            double x_point = i * step_len;
                            double y_point = s(x_point);
                            double x_transformed = x_point * cos(ref_yaw) - y_point * sin(ref_yaw);
                            double y_transformed = x_point * sin(ref_yaw) + y_point * cos(ref_yaw);
                            x_transformed += ref_x;
                            y_transformed += ref_y;
                            next_x_vals.push_back(x_transformed);
                            next_y_vals.push_back(y_transformed);
                        }
                    }
                    json msgJson;
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
