#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;

class Vehicle {
 public:
    
    // Initializes Vehicle
    Vehicle(){}
    ~Vehicle() {}

    Vehicle(int lane, float s, float v, float a, string state="CS");
    
    // ********** WX newly defined method: Start ********************
    
    // car info: unique ID, x, y, x_dot, y_dot, s, d
    Vehicle(vector<double> car_info);
    
    // update vehicle info
    void Update_Vehicle_Info(vector<double> car_info);
    
    // ********** WX newly defined method: End ********************
    
    vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> &predictions);

    vector<string> successor_states();

    vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> &predictions);

    vector<float> get_kinematics(map<int, vector<Vehicle>> &predictions, int lane);

    vector<Vehicle> constant_speed_trajectory();

    vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> &predictions);

    vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> &predictions);

    vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> &predictions);

    void increment(int dt);

    float position_at(int t);

    bool get_vehicle_behind(map<int, vector<Vehicle>> &predictions, int lane,
                          Vehicle &rVehicle);

    bool get_vehicle_ahead(map<int, vector<Vehicle>> &predictions, int lane,
                         Vehicle &rVehicle);

    vector<Vehicle> generate_predictions(int horizon=2);

    void realize_next_state(vector<Vehicle> &trajectory);

    void configure(vector<int> &road_data);

    // public Vehicle variables
    
    struct collider{
    bool collision; // is there a collision?
    int  time; // time collision happens
    };

    map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1},
                                     {"LCR", -1}, {"PLCR", -1}};

    int L = 1;

    int preferred_buffer = 6; // impacts "keep lane" behavior.

    int lane, s, goal_lane, goal_s, lanes_available;

    float v, target_speed, a, max_acceleration;

    string state;
    
    //WX new defined properties
    int ID;
    double x;
    double y;
    double x_dot;
    double y_dot;
    double s2;
    double d2;
    
    double yaw;
    
    vector<double> previous_path_x;
    vector<double> previous_path_y;
    
};

#endif  // VEHICLE_H
