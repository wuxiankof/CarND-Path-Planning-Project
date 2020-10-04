#include "vehicle.h"
#include <iostream>
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "helpers.h"
#include "spline.h"

using std::string;
using std::vector;

Vehicle::Vehicle(int lane, float s, float v, float a, string state){
    
    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;
}

// car info: unique ID, x, y, x_dot, y_dot, s, d
Vehicle::Vehicle(vector<double> car_info){
    
    // using Update_Vehicle_Info() to init the vehicle
    this->Update_Vehicle_Info(car_info);
    
    // set the vehicle ID
    this->ID = (int)car_info[0];
}

// to update vehicle info at each time step
void Vehicle::Update_Vehicle_Info(vector<double> car_info){
    
    this->x = car_info[1];
    this->y = car_info[2];
    this->x_dot = car_info[3];
    this->y_dot = car_info[4];
    this->s = car_info[5];
    this->d = car_info[6];
    
    this->lane = d2LaneNumber(this->d);
    
    this->yaw = 0;
    this->v = 0;
    this->a = 0;
    if (this->x_dot != 0 || this->y_dot != 0){
        this->yaw = atan2(this->y_dot, this->x_dot);  // since this function is only for other vehicles rather than the ego, therefore the unit of yaw is in rad/sec.
        this->v = this->x_dot/cos(this->yaw);
        this->a = (this->v - this->v_prev) / this-> time_per_timestep;
    }
    this->v_prev = this->v;
}

// calculate position of the vehicle at time t.
float Vehicle::position_at(int t) {
    
    float pos = this->s + this->v*t + this->a*t*t/2.0;

    return pos;
}

// generate predictions of paths for other vehicles.
vector<Vehicle> Vehicle::generate_predictions(int timesteps){
    
    // Generates predictions for non-ego vehicles to be used in trajectory
    
    vector<Vehicle> predictions;
    
    for(int i = 0; i < timesteps; ++i) {
        
        float next_s = position_at(i * this->time_per_timestep);
        float next_v = this->v;
        
        predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));
    }

    return predictions;
}

// check if there are adjacent vehicles (both front and back) in a perticular lane
vector<bool> Vehicle::check_others(int lane, map<int, vector<Vehicle>> &predictions, Vehicle &rVehicle){
    
    bool too_close_front = false;
    bool too_close_back = false;
    
    int prev_size = (int) rVehicle.previous_path_x.size();
    
    double car_s = this->s;
    if (prev_size > 0)
        car_s = this->end_path_s;
    
    map<int, vector<Vehicle>>::iterator it;
    for (it = predictions.begin(); it != predictions.end(); ++it){
        
        Vehicle temp_vehicle = it->second[0];  // current TimeStep only
        
        // the vehicle in the lane of the ego
        if (temp_vehicle.lane == lane){
            
            double vx = temp_vehicle.x_dot;
            double vy = temp_vehicle.y_dot;
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = temp_vehicle.s;
            
            // if using previous points can project s value out
            // WX: this is for the other car ! assume the same horizon as the ego
            check_car_s += prev_size * rVehicle.time_per_timestep * check_speed;
            
            if (too_close_front && too_close_back) {
                
                return {too_close_front, too_close_back};
            }
            
            // check s values greater than mine and s gap
            if((check_car_s > car_s) && (check_car_s - car_s < 30)){
            
                too_close_front = true;
            }
            
            if((check_car_s < car_s) && (check_car_s - car_s < 30)){
            
                too_close_back = true;
            }
        }
    }
    
    return {too_close_front, too_close_back};
}

vector<Vehicle> Vehicle::generate_trajectory(map<int, vector<Vehicle>> &predictions) {
    
    // provided code for smooth the path
    int prev_size = (int)this->previous_path_x.size();
    double car_s = this->s;
    
    if (prev_size > 0){
        
        car_s = this->end_path_s;
    }
    
    vector<bool> too_close = this->check_others(this->lane, predictions, *this);
    
    if(too_close[0]){ // front car too close
        
        vector<bool> too_close_left = {true, true};
        vector<bool> too_close_right = {true, true};
        
        if (this->lane > 0)
            too_close_left = this->check_others(this->lane-1, predictions, *this);
        if (this->lane <2)
            too_close_right = this->check_others(this->lane+1, predictions, *this);
        
        if (!too_close_left[0] && !too_close_left[1])
            this->lane --;
        else{
            if (too_close_right[0] && !too_close_right[1])
                this->lane ++;
        }
        
        // calculate d based on the lane number
        this->d = 4 * this->lane + 2;
        
        // increase of speed allowed in each time step
        this->target_speed -= .224;
    }
    else if(this->target_speed < this->speed_limit){
        
        // decrese of speed allowed in each time step
        this->target_speed += .224;
    }
    
    // smooth the path
    
    vector<double> ptsx;
    vector<double> ptsy;
    
    double ref_x = this->x;
    double ref_y = this->y;
    double ref_yaw = deg2rad(this->yaw);
    
    // if previous size is almost empty, use th car as starting reference
    if (prev_size < 2){
        
        // use two points that make the path tangent to the car
        double prev_car_x = ref_x - cos(ref_yaw);
        double prev_car_y = ref_y - sin(ref_yaw);
        
        ptsx.push_back(prev_car_x);
        ptsx.push_back(ref_x);
        
        ptsy.push_back(prev_car_y);
        ptsy.push_back(ref_y);
    }
    else{
        
        // use the previous path's end point as starting reference
        ref_x = this->previous_path_x[prev_size-1];
        ref_y = this->previous_path_y[prev_size-1];
        
        double ref_x_prev = this->previous_path_x[prev_size-2];
        double ref_y_prev = this->previous_path_y[prev_size-2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
        
        // use two points that make the path tangent to the previous path's end point
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);
        
        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }
    
    // In frenet add evenly 30m spaced points ahead of the starting reference
    vector<double> next_wp0 = getXY(car_s+30, this->d, *p_mapPts_s, *p_mapPts_x, *p_mapPts_y);
    vector<double> next_wp1 = getXY(car_s+60, this->d, *p_mapPts_s, *p_mapPts_x, *p_mapPts_y);
    vector<double> next_wp2 = getXY(car_s+90, this->d, *p_mapPts_s, *p_mapPts_x, *p_mapPts_y);
    
    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);
    
    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);
    
    for(int i=0; i<ptsx.size(); i++){
        
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;
        
        //shift car reference angle to 0 degrees: to check math later
        ptsx[i] = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
        ptsy[i] = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);
    }
    
    // create a spline
    tk::spline s;
    
    // set (x,y) points to the spline
    s.set_points(ptsx, ptsy);
    
    // define the actual (x, y) points we will use for the planner
    vector<Vehicle> traj;
    for (int i=0; i< prev_size; i++){
        
        vector<double> car_info = {0, previous_path_x[i], previous_path_y[i], 0, 0, 0, 0};
        traj.push_back(Vehicle(car_info));
    }
    
    // calculate hwo to break up spline points so that we travel at our desired reference v
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x*target_x + target_y*target_y);
    double N = (target_dist/(this->time_per_timestep * this->target_speed));
    
    double x_add_on = 0;
    
    for (int i=0; i<50-prev_size; i++){
        
        double x_point = x_add_on + target_x/N;
        double y_point = s(x_point);
        
        x_add_on = x_point;
        
        double x_ref = x_point;
        double y_ref = y_point;
        
        x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
        y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
        
        x_point += ref_x;
        y_point += ref_y;
        
        vector<double> car_info = {0, x_point, y_point, 0, 0, 0, 0};
        traj.push_back(Vehicle(car_info));
    }

    return traj;
}

