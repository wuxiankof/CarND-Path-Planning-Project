#ifndef ROAD_H
#define ROAD_H

#include <map>
#include <string>
#include <vector>
#include "vehicle.h"

using std::vector;

class Road {

public:
    
    // Initializes Road
    Road(int speed_limit, double traffic_density, vector<int> &lane_speeds) {
      
        this->num_lanes = (int) lane_speeds.size();
        this->lane_speeds = lane_speeds;
        this->speed_limit = speed_limit;
        this->density = traffic_density;
        this->camera_center = this->update_width/2;
    }

    // ********** WX newly defined method: Start ************************************************************
    
    // overloaded constructor
    Road(vector<double> &map_waypoints_x, vector<double> &map_waypoints_y, vector<double> &map_waypoints_s, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy){
      
        mapPts_x = map_waypoints_x;
        mapPts_y = map_waypoints_y;
        mapPts_s = map_waypoints_s;
        mapPts_dx = map_waypoints_dx;
        mapPts_dy = map_waypoints_dy;
        
        // car info: unique ID, x, y, x_dot, y_dot, s, d
        // waymap pt 0: 784.6001 1135.571 0 -0.02359831 -0.9997216
        double d = 6;
        double x = mapPts_x[0] + d * mapPts_dx[0];
        double y = mapPts_y[0] + d * mapPts_dy[0];
        double s = mapPts_s[0];
        
        // init ego_Vehicle
        vector<double> ego_info = {-1, x, y, 0, 0, s, d};
        this->ego_Vehicle = Vehicle(ego_info);
    }

    ~Road() {}

    // Road functions
    void update_ego_vehicle(double x, double y, double s, double d, double yaw, double speed, vector<double> &previous_path_x, vector<double> &previous_path_y);
    
    void update_vehicles(vector <vector<double>> sensor_fusion_data);
    
    void print_all_vehicles();
    
    void get_ego_trajectory(vector<double> &next_x_vals, vector<double> &next_y_vals);
    
    // ********** WX newly defined method: End ************************************************************
    
    Vehicle get_ego();

    void populate_traffic();

    void advance();

    void display(int timestep);

    void add_ego(int lane_num, int s, std::vector<int> &config_data);

    void cull();
    

    // Road variables
    int update_width = 70;

    int vehicles_added = 0;

    int ego_key = -1;

    int num_lanes, speed_limit, camera_center;

    double density;

    std::map<int, Vehicle> vehicles;

    std::string ego_rep = " *** ";

    std::vector<int> lane_speeds;
    
    // ********** WX newly defined variables: Start ********************
    
    vector<double> mapPts_x;
    vector<double> mapPts_y;
    vector<double> mapPts_s;
    vector<double> mapPts_dx;
    vector<double> mapPts_dy;
    
    int iTimeStepCounter;
    
    Vehicle ego_Vehicle;
    
    // ********** WX newly defined variables: End ********************
    
    
};

#endif  // ROAD_H
