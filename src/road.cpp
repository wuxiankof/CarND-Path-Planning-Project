#include <iostream>
#include <iterator>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include "road.h"
#include "vehicle.h"
#include "helpers.h"

using std::map;
using std::string;
using std::vector;

// Initializes Road
Road::Road(int speed_limit, double traffic_density, vector<int> &lane_speeds) {
  
    this->num_lanes = (int) lane_speeds.size();
    this->lane_speeds = lane_speeds;
    this->speed_limit = speed_limit;
    this->density = traffic_density;
    this->camera_center = this->update_width/2;
}


// ********** WX newly defined method: Start *****************************************

// overloaded constructor
Road::Road(vector<double> &map_waypoints_x,
           vector<double> &map_waypoints_y,
           vector<double> &map_waypoints_s,
           vector<double> map_waypoints_dx,
           vector<double> map_waypoints_dy){
  
    mapPts_x = map_waypoints_x;
    mapPts_y = map_waypoints_y;
    mapPts_s = map_waypoints_s;
    mapPts_dx = map_waypoints_dx;
    mapPts_dy = map_waypoints_dy;
    
    // car info: unique ID, x, y, x_dot, y_dot, s, d
    // waymap pt 0: 784.6001 1135.571 0 -0.02359831 -0.9997216
    double d = 6; // lane 2
    double x = mapPts_x[0] + d * mapPts_dx[0];
    double y = mapPts_y[0] + d * mapPts_dy[0];
    double s = mapPts_s[0];
    
    // init ego_Vehicle
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

void Road::update_ego_vehicle(double x, double y, double s, double d, double yaw, double speed, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d){
    
    double speed2 = speed * 0.44704; //WX MPH to m/s 1.6*1000/3600
    
    this->ego_Vehicle.x = x;
    this->ego_Vehicle.y = y;
    this->ego_Vehicle.s = s;
    this->ego_Vehicle.d = d;
    this->ego_Vehicle.yaw = yaw;
    
    this->ego_Vehicle.v = speed2;
    this->ego_Vehicle.a =  (speed2 - this->ego_Vehicle.v_prev) / this->ego_Vehicle.time_per_timestep;
    this->ego_Vehicle.v_prev = speed2;
    
    this->ego_Vehicle.x_dot = speed2 * cos(deg2rad(yaw));
    this->ego_Vehicle.y_dot = speed2 * sin(deg2rad(yaw));
    
    this->ego_Vehicle.previous_path_x = previous_path_x;
    this->ego_Vehicle.previous_path_y = previous_path_y;
    
    this->ego_Vehicle.end_path_s = end_path_s;
    this->ego_Vehicle.end_path_d = end_path_d;
    
    this->ego_Vehicle.lane = d2LaneNumber(d);
    
    this->ego_Vehicle.goal_s = s + this->ego_Vehicle.preferred_buffer + 2 * speed2; // WX: add vhe lengh and 2 sec buffer
}

void Road::update_vehicles(vector <vector<double>> sensor_fusion_data){
    
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

void Road::print_all_vehicles(){
    
    map<int, Vehicle>::iterator it = this->vehicles.begin();
    
    std::cout << "TimeStep="<< this->iTimeStepCounter << ", No of Cars:"<< this->vehicles.size() << std::endl;
    
    //loop all vehicles, print info
    while (it != this->vehicles.end()) {
        
        Vehicle vehicle = (Vehicle)it->second;
        std::cout << vehicle.ID << ", ";
        std::cout << vehicle.x << ", ";
        std::cout << vehicle.y << ", ";
        std::cout << vehicle.x_dot  << ", ";
        std::cout << vehicle.y_dot << ", ";
        std::cout << vehicle.s << ", ";
        std::cout << vehicle.d << ", ";
        std::cout <<"Lane="<< vehicle.lane <<std::endl;
        ++it;
    }

}

void Road::get_ego_trajectory(vector<double> &next_x_vals, vector<double> &next_y_vals){
    
    //ref: advance()
    
    //loop all vehicles, stor predictions
    map<int ,vector<Vehicle> > predictions;
    map<int, Vehicle>::iterator it = this->vehicles.begin();
    while (it != this->vehicles.end()) {
        
        int v_id = it->first;
        vector<Vehicle> preds = it->second.generate_predictions2(); //WX: default: 50 timesteps
        predictions[v_id] = preds;
        ++it;
    }
    
    // update ego_Vehicle's path prediction
    vector<Vehicle> trajectory = this->ego_Vehicle.choose_next_state(predictions);
    
    // update return variables
    for (int i = 0; i < trajectory.size(); ++i) {
        
        Vehicle vehicle = trajectory[i];
        next_x_vals.push_back(vehicle.x);
        next_y_vals.push_back(vehicle.y);
    }
}


// ********** WX newly defined method: End *****************************************

Vehicle Road::get_ego() {
  return this->vehicles.find(this->ego_key)->second;
}


void Road::populate_traffic() {
    
    int start_s = std::max(this->camera_center - (this->update_width/2), 0);

    for (int l = 0; l < this->num_lanes; ++l) {
        
        int lane_speed = this->lane_speeds[l];
        bool vehicle_just_added = false;

        for (int s = start_s; s < start_s+this->update_width; ++s) {
            if (vehicle_just_added) {
                vehicle_just_added = false;
            }

            if (((double) rand() / (RAND_MAX)) < this->density) {
                Vehicle vehicle = Vehicle(l,s,lane_speed,0);
                vehicle.state = "CS";
                this->vehicles_added += 1;
                this->vehicles.insert(std::pair<int,Vehicle>(vehicles_added,vehicle));
                vehicle_just_added = true;
            }
        }
    }
}

void Road::advance() {
    
    map<int ,vector<Vehicle> > predictions;

    map<int, Vehicle>::iterator it = this->vehicles.begin();
    
    //loop all vehicles, stor predictions
    while (it != this->vehicles.end()) {
        
        int v_id = it->first;
        vector<Vehicle> preds = it->second.generate_predictions();
        predictions[v_id] = preds;
        ++it;
    }
  
    it = this->vehicles.begin();

    while (it != this->vehicles.end()) {
        
        int v_id = it->first;
        
        //note: ego_key = -1 already difined
        if (v_id == ego_key) {
            vector<Vehicle> trajectory = it->second.choose_next_state(predictions);
            it->second.realize_next_state(trajectory);
        } else {
            it->second.increment(1);
        }
        
        ++it;
    }
}

void Road::add_ego(int lane_num, int s, vector<int> &config_data) {
    
  map<int, Vehicle>::iterator it = this->vehicles.begin();

  while (it != this->vehicles.end()) {
    int v_id = it->first;
    Vehicle v = it->second;
    if (v.lane == lane_num && v.s == s) {
      this->vehicles.erase(v_id);
    }
    ++it;
  }
    
  Vehicle ego = Vehicle(lane_num, s, this->lane_speeds[lane_num], 0);
  ego.configure(config_data);
  ego.state = "KL";
  this->vehicles.insert(std::pair<int,Vehicle>(ego_key,ego));
}

void Road::display(int timestep) {
  Vehicle ego = this->vehicles.find(this->ego_key)->second;
  int s = ego.s;
  string state = ego.state;

  this->camera_center = std::max(s, this->update_width/2);
  int s_min = std::max(this->camera_center - this->update_width/2, 0);
  int s_max = s_min + this->update_width;

  vector<vector<string>> road;

  for (int i = 0; i < this->update_width; ++i) {
    vector<string> road_lane;
    for (int ln = 0; ln < this->num_lanes; ++ln) {
      road_lane.push_back("     ");
    }
    road.push_back(road_lane);
  }

  map<int, Vehicle>::iterator it = this->vehicles.begin();

  while (it != this->vehicles.end()) {
    int v_id = it->first;
    Vehicle v = it->second;

    if (s_min <= v.s && v.s < s_max) {
      string marker = "";

      if (v_id == this->ego_key) {
        marker = this->ego_rep;
      } else {
        std::stringstream oss;
        std::stringstream buffer;
        buffer << " ";
        oss << v_id;

        for (int buffer_i = oss.str().length(); buffer_i < 3; ++buffer_i) {
          buffer << "0";
        }
        buffer << oss.str() << " ";
        marker = buffer.str();
      }
      road[int(v.s - s_min)][int(v.lane)] = marker;
    }
    ++it;
  }
    
  std::ostringstream oss;
  oss << "+Meters ======================+ step: " << timestep << std::endl;
  int i = s_min;

  for (int lj = 0; lj < road.size(); ++lj) {
    if (i%20 ==0) {
      std::stringstream buffer;
      std::stringstream dis;
      dis << i;
      
      for (int buffer_i = dis.str().length(); buffer_i < 3; ++buffer_i) {
        buffer << "0";
      }
      
      oss << buffer.str() << dis.str() << " - ";
    } else {
      oss << "      ";
    }
    ++i;
    for (int li = 0; li < road[0].size(); ++li) {
      oss << "|" << road[lj][li];
    }
      oss << "|";
      oss << "\n";
  }

  std::cout << oss.str();
}
