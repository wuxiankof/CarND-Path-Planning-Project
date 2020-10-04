#include <iostream>
#include <iterator>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include "pathplanner.h"
#include "vehicle.h"
#include "helpers.h"

using std::map;
using std::string;
using std::vector;

// Initializes 

PathPlanner::PathPlanner(vector<double> &map_waypoints_x,
           vector<double> &map_waypoints_y,
           vector<double> &map_waypoints_s,
           vector<double> map_waypoints_dx,
           vector<double> map_waypoints_dy){
  
    mapPts_x = map_waypoints_x;
    mapPts_y = map_waypoints_y;
    mapPts_s = map_waypoints_s;
    mapPts_dx = map_waypoints_dx;
    mapPts_dy = map_waypoints_dy;
    
    // take the 1st point of map way points
    double d = 6; // middle of lane 1
    double x = mapPts_x[0] + d * mapPts_dx[0];
    double y = mapPts_y[0] + d * mapPts_dy[0];
    double s = mapPts_s[0];
    
    // init ego_Vehicle
    // car info: unique ID, x, y, x_dot, y_dot, s, d
    vector<double> ego_info = {-1, x, y, 0, 0, s, d};
    this->ego_Vehicle = Vehicle(ego_info);
    this->ego_Vehicle.v = 0;
    this->ego_Vehicle.v_prev = 0;
    this->ego_Vehicle.a = 0;
    this->ego_Vehicle.target_speed = 0;
    
    this->ego_Vehicle.p_mapPts_x = & mapPts_x;
    this->ego_Vehicle.p_mapPts_y = & mapPts_y;
    this->ego_Vehicle.p_mapPts_s = & mapPts_s;
}

// to update the variable `ego_Vehicle` with the latest location/kinematics.
void PathPlanner::update_ego_vehicle(double x, double y, double s, double d, double yaw, double speed, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d){
    
    // convert MPH to m/s, i.e., 1.6*1000/3600=0.44704
    double speed2 = speed * 0.44704;
    
    // update ego's x, y, s, d, yaw
    this->ego_Vehicle.x = x;
    this->ego_Vehicle.y = y;
    this->ego_Vehicle.s = s;
    this->ego_Vehicle.d = d;
    this->ego_Vehicle.yaw = yaw;
    
    // update eog's v and a
    this->ego_Vehicle.v = speed2;
    this->ego_Vehicle.a =  (speed2 - this->ego_Vehicle.v_prev) / this->ego_Vehicle.time_per_timestep;
    this->ego_Vehicle.v_prev = speed2;
    
    // convert d to lane number
    this->ego_Vehicle.lane = d2LaneNumber(d);
    
    // convert speed2 to x_dot and y_dot
    this->ego_Vehicle.x_dot = speed2 * cos(deg2rad(yaw));
    this->ego_Vehicle.y_dot = speed2 * sin(deg2rad(yaw));
    
    // store previous path info
    this->ego_Vehicle.previous_path_x = previous_path_x;
    this->ego_Vehicle.previous_path_y = previous_path_y;
    this->ego_Vehicle.end_path_s = end_path_s;
    this->ego_Vehicle.end_path_d = end_path_d;
}

//to update all vehicle objects with the sensor fusion data stored in the map variable `vehicles`.
void PathPlanner::update_vehicles(vector <vector<double>> sensor_fusion_data){
    
    for (int i=0; i < sensor_fusion_data.size(); i++){
                   
        vector<double> car_info = sensor_fusion_data[i];
        int ID = car_info[0];
        if (this->vehicles.count(ID)){
            this->vehicles[ID].Update_Vehicle_Info(car_info);
        }
        else{
            Vehicle vehicle = Vehicle(car_info);
            this->vehicles.insert(std::pair<int,Vehicle>(ID, vehicle));
        }
    }
}

//to obtain the planned tragectory which contains coordinates of points in the next 50 time steps.
void PathPlanner::get_ego_trajectory(vector<double> &next_x_vals, vector<double> &next_y_vals){
    

    //loop all vehicles, store predictions
    map<int ,vector<Vehicle> > predictions;
    map<int, Vehicle>::iterator it = this->vehicles.begin();
    while (it != this->vehicles.end()) {
        
        int v_id = it->first;
        vector<Vehicle> preds = it->second.generate_predictions(); //WX: default: 50 timesteps
        predictions[v_id] = preds;
        ++it;
    }
    
    // update ego_Vehicle's path prediction
    vector<Vehicle> trajectory = this->ego_Vehicle.generate_trajectory(predictions);
    
    // update return variables
    for (int i = 0; i < trajectory.size(); ++i) {
        
        Vehicle vehicle = trajectory[i];
        next_x_vals.push_back(vehicle.x);
        next_y_vals.push_back(vehicle.y);
    }
}
