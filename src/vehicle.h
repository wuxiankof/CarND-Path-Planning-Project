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

    // car info: unique ID, x, y, x_dot, y_dot, s, d
    Vehicle(vector<double> car_info);
    
    // update vehicle info
    void Update_Vehicle_Info(vector<double> car_info);
  
    // calculate position of the vehicle at time t
    float position_at(int t);
    
    // generate predictions of paths for other vehicles
    vector<Vehicle> generate_predictions(int timesteps=50);

    // check if there are adjacent vehicles (both front and back) in a perticular lane
    vector<bool> check_others(int lane, map<int, vector<Vehicle>> &predictions, Vehicle &rVehicle);
    
    // generate trajectory
    vector<Vehicle> generate_trajectory(map<int, vector<Vehicle>> &predictions);

    // public Vehicle variables
    
    // time step of the simulaiton, which is 0.02 seconds per timestep as provided
    double time_per_timestep = 0.02;
    
    // ID, lane number
    int ID, lane;
    
    // kinematics
    double x, y, x_dot, y_dot, s, d;
    double v, v_prev, a;
    double yaw; //if it's for the ego vehicle, the unit is in degree/sec; otherwise, the unit is in rad/sec.
    
    // speed the vehicle is targeting to
    float target_speed;
    
    // speed limit 49.5 MPH, MPH to m/s 1.6*1000/3600 or 1/2.24
    float speed_limit = 49.5 * 0.44704;
    
    // state of the vehicle
    // currently only the default state "CS"
    // will be expanded to other states in a later time (not in this submission)
    string state;

    vector<double> previous_path_x;
    vector<double> previous_path_y;
    
    double end_path_s;
    double end_path_d;
    
    vector<double> * p_mapPts_x;
    vector<double> * p_mapPts_y;
    vector<double> * p_mapPts_s;
};

#endif  // VEHICLE_H
