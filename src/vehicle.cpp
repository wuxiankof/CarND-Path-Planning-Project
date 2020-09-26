#include "vehicle.h"
#include <iostream>
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "cost.h"
#include "helpers.h"

using std::string;
using std::vector;

Vehicle::Vehicle(int lane, float s, float v, float a, string state){
    
    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;
    // this->max_acceleration = -1; // WX: already difnied in .h file
}

// ********** WX newly defined method: Start ********************

// car info: unique ID, x, y, x_dot, y_dot, s, d
Vehicle::Vehicle(vector<double> car_info){
    
    this->ID = (int)car_info[0];
    this->x = car_info[1];
    this->y = car_info[2];
    this->x_dot = car_info[3];
    this->y_dot = car_info[4];
    this->s2 = car_info[5];
    this->d2 = car_info[6];
    
    this->lane = d2LaneNumber(this->d2);
    this->s = this->s2;
    
    // this->max_acceleration = -1; // WX: already difnied in .h file
}


void Vehicle::Update_Vehicle_Info(vector<double> car_info){
    
    this->x = car_info[1];
    this->y = car_info[2];
    this->x_dot = car_info[3];
    this->y_dot = car_info[4];
    this->s2 = car_info[5];
    this->d2 = car_info[6];
    
    this->lane = d2LaneNumber(this->d2);
    this->yaw = atan2(this->y_dot, this->x_dot);  //WX: here for other vehicles, in radian; for ego, should be in degree.
    this->s = this->s2;
    this->v = this->x_dot/cos(this->yaw); //WX: to refer to the sketch
    
    this->a = (this->v - this->v_prev) / this-> time_per_timestep;
    
    if (this->a > this->max_acceleration)
        this->a = this->max_acceleration;
    else if (this->a < -1 * this->max_acceleration)
        this->a = -1 * this->max_acceleration;
    
    this->v_prev = this->v;
}

vector<Vehicle> Vehicle::generate_predictions2(int timesteps){
    
    // Generates predictions for non-ego vehicles to be used in trajectory
    //   generation for the ego vehicle.
    
    vector<Vehicle> predictions;
    
    for(int i = 0; i < timesteps; ++i) {
        
        float next_s = position_at(i * this->time_per_timestep);
        float next_v = this->v;
        
        /* WX: why need this?
        float next_v = 0;
        if (i < timesteps - 1) {
            next_v = position_at( (i+1) * time_per_timestep) - s;
        }
         */
        
        predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));
    }

    return predictions;
}

vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> &predictions, int lane) {
  
    // Gets next timestep kinematics (position, velocity, acceleration)
    //   for a given lane. Tries to choose the maximum velocity and acceleration,
    //   given other vehicle positions and accel/velocity constraints.
    
    float max_velocity_accel_limit = this->max_acceleration + this->v; //WX: curr v + 1 * a
    float new_position;
    float new_velocity;
    float new_accel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
        
        if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
          
          // must travel at the speed of traffic, regardless of preferred buffer
          new_velocity = vehicle_ahead.v;
        }
        else {
            
          float max_velocity_in_front = (vehicle_ahead.s - this->s
                                      - this->preferred_buffer) + vehicle_ahead.v
                                      - 0.5 * (this->a);
          new_velocity = std::min(std::min(max_velocity_in_front,
                                           max_velocity_accel_limit),
                                           this->target_speed);
        }
    }
    else {
        
        new_velocity = std::min(max_velocity_accel_limit, this->target_speed);
    }

    new_accel = new_velocity - this->v; // Equation: (v_1 - v_0)/t = acceleration
    
    // WX added
    if (new_accel > this->max_acceleration)
        new_accel = this->max_acceleration;
    else if (new_accel < -1 * this->max_acceleration)
        new_accel = -1 * this->max_acceleration;
    
    new_position = this->s + new_velocity + new_accel / 2.0;

    return{new_position, new_velocity, new_accel};
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> &predictions) {
  
    //  WX code
    
    if (this->a > this->max_acceleration)
        this->a = this->max_acceleration;
    else if (this->a < -1 * this->max_acceleration)
        this->a = -1 * this->max_acceleration;
    
    vector<Vehicle> traj;
    
    double s_start, s_dot_start, s_dot2_start;
    double s_goal, s_dot_goal, s_dot2_goal;
    // double d_start, d_dot_start, d_dot2_start;
    // double d_goal, d_dot_goal, d_dot2_goal;

    int path_size = (int)this->previous_path_x.size();
    
    // 1. get s_start, etc.
    if (path_size < 2) {
        
        s_start = this->s2;
        s_dot_start = this->v;
        s_dot2_start = this->a;
        
    } else {
        
        //debug
        std::cout << "previous path size >=2 " << std::endl;
        
        double angle = atan2(previous_path_y[1]-previous_path_y[0], previous_path_x[1]-previous_path_x[0]);
        vector<double> fr0 = getFrenet(previous_path_x[0], previous_path_y[0], angle, *this->p_mapPts_x, *this->p_mapPts_y);
        vector<double> fr1 = getFrenet(previous_path_x[1], previous_path_y[1], angle, *this->p_mapPts_x, *this->p_mapPts_y);
        
        s_start = fr0[0];
        s_dot_start = this->v; //(fr1[0] - fr0[0]) / this->time_per_timestep;
        s_dot2_start = this->a;
        
        //d_start = fr[1];
    }
  
    // testing
    s_start = this->s2;
    s_dot_start = this->v;
    s_dot2_start = this->a;

    
    // 2. get s_goal, d_goal
    vector<float> kinematics = get_kinematics(predictions, this->lane);
    s_goal = kinematics[0];  //end of next 1 sec
    s_dot_goal = kinematics[1];
    s_dot2_goal = kinematics[2];
    
    // 3. solve JMT
    vector<double> start = {s_start, s_dot_start, s_dot2_start};
    vector<double> end = {s_goal, s_dot_goal, s_dot2_goal};
    vector<double> jmt = JMT(start, end, 2);
  
    // debug
    std::cout << "jmt input =" << s_start << "," << s_dot_start << "," << s_dot2_start << "," << s_goal << "," << s_dot_goal << "," << s_dot2_goal << std::endl;
    
    //4. generate pts
    int pts_count = (int) 2 / this->time_per_timestep;
    for (int i = 0; i < pts_count; ++i){
        
        double t = this->time_per_timestep * (i+1);
        double s = jmt[0] + jmt[1] * t + jmt[2] * t*t + jmt[3] * t*t*t + jmt[4] * t*t*t*t + jmt[5] * t*t*t*t*t;
        
          //debug
          std::cout << s << ", ";
      
        vector<double> XY = getXY(s, this->d2, *this->p_mapPts_s, *this->p_mapPts_x, *this->p_mapPts_y);
        
        vector<double> car_info = {0, XY[0], XY[1], 0, 0, 0, 0};
        traj.push_back(Vehicle(car_info));
    }
    
    std::cout << std::endl;
    return traj;
}

// ********** WX newly defined method: End ********************

vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> &predictions) {
  
    
    // car info: unique ID, x, y, x_dot, y_dot, s, d

    // WX: testing code 1: strait line
    /*
    double dist_inc = 0.5;
    vector<Vehicle> traj;
    for (int i = 0; i < 50; ++i) {
        
        double x = this->x + (dist_inc*i) * cos(deg2rad(this->yaw));
        double y = this->y + (dist_inc*i) * sin(deg2rad(this->yaw));
                                              
        vector<double> car_info = {0, x, y, 0, 0, 0, 0};
        traj.push_back(Vehicle(car_info));
    }
    
    return traj;
    
    // WX: testing code 2: curved line
    double pos_x;
    double pos_y;
    double angle;
    
    vector<Vehicle> traj;

    int path_size = (int)this->previous_path_x.size();

    for (int i = 0; i < path_size; ++i) {
        
        double x = this->previous_path_x[i];
        double y = this->previous_path_y[i];
        vector<double> car_info = {0, x, y, 0, 0, 0, 0};
        traj.push_back(Vehicle(car_info));
    }

    if (path_size == 0) {
        
        pos_x = this->x;
        pos_y = this->y;
        angle = deg2rad(this->yaw);
        
    } else {
      
        pos_x = this->previous_path_x[path_size-1];
        pos_y = this->previous_path_y[path_size-1];

        double pos_x2 = this->previous_path_x[path_size-2];
        double pos_y2 = this->previous_path_y[path_size-2];
        
        angle = atan2(pos_y-pos_y2, pos_x-pos_x2);
    }

    double dist_inc = 0.5;
    for (int i = 0; i < 50-path_size; ++i) {
        
        double x = pos_x + (dist_inc) * cos(angle+(i+1)*(pi()/100));
        double y = pos_y + (dist_inc) * sin(angle+(i+1)*(pi()/100));
        pos_x += (dist_inc) * cos(angle+(i+1)*(pi()/100));
        pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
        
        vector<double> car_info = {0, x, y, 0, 0, 0, 0};
        traj.push_back(Vehicle(car_info));
    }

    return traj;
     */
    
    //WX test: only do "KL"
    vector<Vehicle> traj = generate_trajectory("KL", predictions);
    
    return traj;
     

    
    //only consider states which can be reached from current FSM state.
    vector<string> possible_successor_states = this->successor_states();
    
    // keep track of the total cost of each state.
    map<string, float> costs;
    
    // loop possible states to get corresponding cost
    for (int i=0; i<possible_successor_states.size(); i++){
        
        // generate a rough idea of what trajectory we would follow IF we chose this state.
        string state = possible_successor_states[i];
        vector<Vehicle> trajectory_for_state = generate_trajectory(state, predictions);
        
        if (trajectory_for_state.size() != 0){
            // calculate the "cost" associated with that trajectory.
            float cost_for_state = calculate_cost(*this, predictions, trajectory_for_state);
            costs[state] = cost_for_state;
        }
    }
    
    // Find the minimum cost state.
    string best_next_state = "KL";
    float min_cost = 9999999;
    
    map<string, float>::iterator it = costs.begin();
    while (it != costs.end()) {
        
        string state = it->first;
        float cost = it->second;
        if (cost < min_cost){
            min_cost = cost;
            best_next_state = state;
        }
            
        ++it;
    }

    return generate_trajectory(best_next_state, predictions);
    
    /* given solution:
    
    vector<string> states = successor_states();
      float cost;
      vector<float> costs;
      vector<vector<Vehicle>> final_trajectories;

      for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
        vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
        if (trajectory.size() != 0) {
          cost = calculate_cost(*this, predictions, trajectory);
          costs.push_back(cost);
          final_trajectories.push_back(trajectory);
        }
      }

      vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
      int best_idx = distance(begin(costs), best_cost);


      return final_trajectories[best_idx];
    */
}

vector<string> Vehicle::successor_states() {
    
  // Provides the possible next states given the current state for the FSM
  //   discussed in the course, with the exception that lane changes happen
  //   instantaneously, so LCL and LCR can only transition back to KL.
  // WX: in the real project, the last assumption above may need to change.
    
    vector<string> states;
    states.push_back("KL");
    string state = this->state;
    
    if(state.compare("KL") == 0) {
        states.push_back("PLCL");
        states.push_back("PLCR");
    }
    else if (state.compare("PLCL") == 0) {
        if (lane != lanes_available - 1) {
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    }
    else if (state.compare("PLCR") == 0) {
        if (lane != 0) {
            states.push_back("PLCR");
            states.push_back("LCR");
        }
    }

    // If state is "LCL" or "LCR", then just return "KL"
    return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state,
                                             map<int, vector<Vehicle>> &predictions) {
    // Given a possible next state, generate the appropriate trajectory to realize
    //   the next state.
    vector<Vehicle> trajectory;

    if (state.compare("CS") == 0) {
        trajectory = constant_speed_trajectory();
    } else if (state.compare("KL") == 0) {
        trajectory = keep_lane_trajectory(predictions);
    } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        trajectory = lane_change_trajectory(state, predictions);
    } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        trajectory = prep_lane_change_trajectory(state, predictions);
    }

    return trajectory;
}



vector<Vehicle> Vehicle::constant_speed_trajectory() {
  // Generate a constant speed trajectory.
  float next_pos = position_at(1);
  vector<Vehicle> trajectory = {Vehicle(this->lane,this->s,this->v,this->a,this->state),
                                Vehicle(this->lane,next_pos,this->v,0,this->state)};
  return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state,
                                                     map<int, vector<Vehicle>> &predictions) {
  // Generate a trajectory preparing for a lane change.
  float new_s;
  float new_v;
  float new_a;
  Vehicle vehicle_behind;
  int new_lane = this->lane + lane_direction[state];
  vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a,
                                        this->state)};
  vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

  if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
    // Keep speed of current lane so as not to collide with car behind.
    new_s = curr_lane_new_kinematics[0];
    new_v = curr_lane_new_kinematics[1];
    new_a = curr_lane_new_kinematics[2];
  } else {
    vector<float> best_kinematics;
    vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
    // Choose kinematics with lowest velocity.
    if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
      best_kinematics = next_lane_new_kinematics;
    } else {
      best_kinematics = curr_lane_new_kinematics;
    }
    new_s = best_kinematics[0];
    new_v = best_kinematics[1];
    new_a = best_kinematics[2];
  }

  trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));
  
  return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state,
                                                map<int, vector<Vehicle>> &predictions) {
  // Generate a lane change trajectory.
  int new_lane = this->lane + lane_direction[state];
  vector<Vehicle> trajectory;
  Vehicle next_lane_vehicle;
  // Check if a lane change is possible (check if another vehicle occupies
  //   that spot).
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
       it != predictions.end(); ++it) {
    next_lane_vehicle = it->second[0];
    if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane) {
      // If lane change is not possible, return empty trajectory.
      return trajectory;
    }
  }
  trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a,
                               this->state));
  vector<float> kinematics = get_kinematics(predictions, new_lane);
  trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1],
                               kinematics[2], state));
  return trajectory;
}

void Vehicle::increment(int dt = 1) {
  this->s = position_at(dt);
}

float Vehicle::position_at(int t) {
    
    float pos = this->s + this->v*t + this->a*t*t/2.0;

    return pos;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> &predictions,
                                 int lane, Vehicle &rVehicle) {
  
    // Returns a true if a vehicle is found behind the current vehicle, false
    //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
    int max_s = -1;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    
    map<int, vector<Vehicle>>::iterator it;
    for (it = predictions.begin(); it != predictions.end(); ++it) {
        
        temp_vehicle = it->second[0];
        
        if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s) {
            max_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }

    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> &predictions, int lane, Vehicle &rVehicle) {
    // Returns a true if a vehicle is found ahead of the current vehicle, false
    //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
    
    int min_s = this->goal_s;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    
    map<int, vector<Vehicle>>::iterator it;
    for (it = predictions.begin(); it != predictions.end(); ++it){
        
        temp_vehicle = it->second[0];  //WX: current TimeStep, see function below
        
        if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s && temp_vehicle.s < min_s) {
            
            min_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }

    return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
    
    // Generates predictions for non-ego vehicles to be used in trajectory
    //   generation for the ego vehicle.
    
    vector<Vehicle> predictions;
    
    for(int i = 0; i < horizon; ++i) {
        
        float next_s = position_at(i);
        float next_v = 0;
        if (i < horizon-1) {
            next_v = position_at(i+1) - s;
        }
        predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));
    }

    return predictions;
}

void Vehicle::realize_next_state(vector<Vehicle> &trajectory) {
  // Sets state and kinematics for ego vehicle using the last state of the trajectory.
  Vehicle next_state = trajectory[1];
  this->state = next_state.state;
  this->lane = next_state.lane;
  this->s = next_state.s;
  this->v = next_state.v;
  this->a = next_state.a;
}

void Vehicle::configure(vector<int> &road_data) {
  // Called by simulator before simulation begins. Sets various parameters which
  //   will impact the ego vehicle.
  target_speed = road_data[0];
  lanes_available = road_data[1];
  goal_s = road_data[2];
  goal_lane = road_data[3];
  max_acceleration = road_data[4];
}
