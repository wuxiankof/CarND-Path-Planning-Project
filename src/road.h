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
    Road(int speed_limit, double traffic_density, vector<int> &lane_speeds);
    ~Road() {}

    // ********** WX newly defined method: Start *******************************************
    
    // overloaded constructor
    Road(vector<double> &map_waypoints_x, vector<double> &map_waypoints_y, vector<double> &map_waypoints_s, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy);

    // Road functions
    void update_ego_vehicle(double x, double y, double s, double d, double yaw, double speed, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d);
    
    void update_vehicles(vector <vector<double>> sensor_fusion_data);
    
    void print_all_vehicles();
    
    void get_ego_trajectory(vector<double> &next_x_vals, vector<double> &next_y_vals);
    
    // ********** WX newly defined method: End ***********************************************
    
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
