# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)

[distance_without_incident]: ./images/distance_without_incident.png "Distance without incident"

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

## Rubric Points

[rubric points](https://review.udacity.com/#!/rubrics/1020/view)

### The code compiles correctly

Code compiles with `cmake` and `make` without errors.

### The car is able to drive at least 4.32 miles without incident

Yes, the car can drive much more than 4.32 miles without incident. 

![Distance without incident][distance_without_incident]

### The car drives according to the speed limit

Yes, the car strictly follows speed limit. The maximum speed it drives is set to 49.5 mph.

### Max Acceleration and Jerk are not Exceeded

Yes, max acceleration and jerk are not exceeded. The warning never shows.

### Car does not have collisions

No collision happens.

### The car stays in its lane, except for the time between changing lanes

Yes, the car stays in lane except for changing lanes.

### The car is able to change lanes

The car will change lane smoothly when it is behind a slower vehicle and safe to change.

## Reflection

### Prediction

The sensor fusion contains all the information (id, x, y, vx, vy, s, d) about cars on the same side of ego vehicle. The velocity "vx", "vy" and "s" are used to predict where the car will be in the future.

The getVehicleAhead() and getVehicleBehind() use prediction to find vechiles ahead and behind (main.cpp, lines 167 to 213).

```
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

```

### Behavior Planning

This part is to decide what to do next. With current state and sensor fusion data, calculate velocity and lane on which the car shall drive.

When changing lane, the car will slow down when it is too close to vehicle ahead on the same lane, or the change-to lane (main.cpp, lines 336 to 356).

```
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
    }
```

If not changing lane, increase velocity if there is no vehicle ahead, or it is far enough (main.cpp, lines 357 to 365).

```
    if (vehicle_ahead.size() == 0) {
        // No vehicle ahead
        ref_velocity += default_acceleration;
    } else {
        double v = vehicle_ahead[0];
        double s = vehicle_ahead[1];
        if (s - end_path_s > DEFAULT_BUFFER_ZONE) {
            ref_velocity += default_acceleration;
        }
    }
```

If the car is too close to vehicle ahead, check whether it is safe to change lane. If yes, find the better one. Otherwise, keep lane and slow down (main.cpp, lines 365 to 372).

```
    if (car_speed > v) {
        int lane_to_change = findLaneToChange(sensor_fusion, car_lane, end_path_s, pred_time);
        if (lane_to_change != -1) {
            next_lane = lane_to_change;
            cout << "car_lane: " << car_lane << "; next_lane: " << next_lane << endl;
        } else {
            ref_velocity -= default_acceleration;
        }
    }
```

### Trajectory Generation

The previous path points left over from last cycle will be used when generate trajectory, and new way points will be appended until it has 50 total points (main.cpp, lines 425 to 448). Using information from the previous path ensures that there is a smooth transition from cycle to cycle. 

```
    // Insert previous path points left over from last cycle
    for(int i = 0; i < prev_size; i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }
    
    // Append new way points
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);
    double N = target_dist / (0.02 * ref_velocity);
    double step_len = target_dist / N;
    for(int i = 1; i <= 50 - prev_size; i++)
    {
        double x_point = i * step_len;
        double y_point = s(x_point);
        // Transform to global coordinate system
        double x_transformed = x_point * cos(ref_yaw) - y_point * sin(ref_yaw);
        double y_transformed = x_point * sin(ref_yaw) + y_point * cos(ref_yaw);
        x_transformed += ref_x;
        y_transformed += ref_y;
        next_x_vals.push_back(x_transformed);
        next_y_vals.push_back(y_transformed);
    }
```
