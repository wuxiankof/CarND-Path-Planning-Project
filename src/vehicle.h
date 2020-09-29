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
    
    // ********** WX newly defined method: Start ******************************************
    
    
    // car info: unique ID, x, y, x_dot, y_dot, s, d
    Vehicle(vector<double> car_info);
    
    // update vehicle info
    void Update_Vehicle_Info(vector<double> car_info);
    
    // generate predictions for other vehicles
    vector<Vehicle> generate_predictions2(int timesteps=50);
    
    vector<float> get_kinematics(map<int, vector<Vehicle>> &predictions, int lane, double T=1);
    
    bool check_ahead(int lane, map<int, vector<Vehicle>> &predictions, Vehicle &rVehicle);
        
    vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> &predictions);
    
    vector<Vehicle> get_trajectory_from_JMT(vector<double> jmt, double T, bool bPrint=true);
    
    // ********** WX newly defined method: End ********************************************
    
    vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> &predictions);

    vector<string> successor_states();

    vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> &predictions);

    vector<Vehicle> constant_speed_trajectory();


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

    float v, a;
    
    float speed_limit = 49.5 * 0.44704; //WX MPH to m/s 1.6*1000/3600 or 1/2.24
    float max_acceleration = 10; //WX: as required: total acceleration not over 10 m/s^2
    
    float target_speed;

    string state;
    
    //WX new defined properties
    int ID;
    
    double x;
    double y;
    double x_dot;
    double y_dot;
    //double s2;
    double d;
    
    double yaw; //WX: for other vehicles, in radian; for ego, should be in degree.
    
    vector<double> previous_path_x;
    vector<double> previous_path_y;
    
    double end_path_s;
    double end_path_d;
    
    vector<double> * p_mapPts_x;
    vector<double> * p_mapPts_y;
    vector<double> * p_mapPts_s;
    
    double time_per_timestep = 0.02;   // 0.02 seconds per timestep
    double v_prev;
    
};

#endif  // VEHICLE_H
