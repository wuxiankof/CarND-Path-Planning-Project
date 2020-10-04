#ifndef ROAD_H
#define ROAD_H

#include <map>
#include <string>
#include <vector>
#include "vehicle.h"

using std::vector;

class PathPlanner {

public:
    
    // Initializes 
   
    PathPlanner(vector<double> &map_waypoints_x, vector<double> &map_waypoints_y, vector<double> &map_waypoints_s, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy);
    ~PathPlanner() {}

    //  functions
    
    // to update the variable `ego_Vehicle` with the latest location/kinematics.
    void update_ego_vehicle(double x, double y, double s, double d, double yaw, double speed, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d);
    
    // to update all vehicle objects with the sensor fusion data stored in the map variable
    void update_vehicles(vector <vector<double>> sensor_fusion_data);
    
    // to obtain the planned tragectory which contains coordinates of points in the next 50 time steps
    void get_ego_trajectory(vector<double> &next_x_vals, vector<double> &next_y_vals);

    //  variables
    Vehicle ego_Vehicle; // the self-driving car
    std::map<int, Vehicle> vehicles; // all other vehicles

    vector<double> mapPts_x;
    vector<double> mapPts_y;
    vector<double> mapPts_s;
    vector<double> mapPts_dx;
    vector<double> mapPts_dy;
};

#endif  // ROAD_H
