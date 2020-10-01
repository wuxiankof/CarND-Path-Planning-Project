#include "vehicle.h"
#include <iostream>
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "cost.h"
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
    this->s = car_info[5];
    this->d = car_info[6];
    
    this->lane = d2LaneNumber(this->d);
    this->a = 0;
    
    if (this->x_dot != 0 || this->y_dot!=0){
        this->yaw = atan2(this->y_dot, this->x_dot);  //WX: here for other vehicles, in radian; for ego, should be in degree.
        this->v = this->x_dot/cos(this->yaw); //WX: to refer to the sketch
        this->v_prev = this->v;
    }
}


void Vehicle::Update_Vehicle_Info(vector<double> car_info){
    
    this->x = car_info[1];
    this->y = car_info[2];
    this->x_dot = car_info[3];
    this->y_dot = car_info[4];
    this->s = car_info[5];
    this->d = car_info[6];
    
    this->lane = d2LaneNumber(this->d);
    
    if (this->x_dot != 0 || this->y_dot!=0){
        this->yaw = atan2(this->y_dot, this->x_dot);  //WX: here for other vehicles, in radian; for ego, should be in degree.
        this->v = this->x_dot/cos(this->yaw); //WX: to refer to the sketch
        this->a = (this->v - this->v_prev) / this-> time_per_timestep;
    
        if (this->a > this->max_acceleration)
            this->a = this->max_acceleration;
        else if (this->a < -1 * this->max_acceleration)
            this->a = -1 * this->max_acceleration;
    
        this->v_prev = this->v;
    }
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

vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> &predictions, int lane, double T) {
  
    // Gets next timestep kinematics (position, velocity, acceleration)
    //   for a given lane. Tries to choose the maximum velocity and acceleration,
    //   given other vehicle positions and accel/velocity constraints.
    
    float max_velocity_accel_limit = this->v + this->max_acceleration * 0.9 * T; //WX: curr v + a * T
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
            max_velocity_in_front *= T; // WX: to reflect the acture horizon
            
            new_velocity = std::min(std::min(max_velocity_in_front,
                                           max_velocity_accel_limit),
                                           this->target_speed);
        }
    }
    else {
        
        new_velocity = std::min(max_velocity_accel_limit, this->target_speed);
    }

    
    //** test: let new_velocity just = this->target_speed
    new_velocity = std::min(max_velocity_accel_limit, this->target_speed);
    
    new_accel = (new_velocity - this->v) / T; // Equation: (v_1 - v_0)/t = acceleration
    
    // WX added
    if (new_accel > this->max_acceleration)
        new_accel = this->max_acceleration;
    else if (new_accel < -1 * this->max_acceleration)
        new_accel = -1 * this->max_acceleration;
    
    new_position = this->s + new_velocity*T + new_accel*T*T / 2.0;
    //float pos = this->s + this->v*t + this->a*t*t/2.0;

    return{new_position, new_velocity, new_accel};
}

vector<bool> Vehicle::check_others(int lane, map<int, vector<Vehicle>> &predictions, Vehicle &rVehicle){
  
    // Returns a true if a vehicle is found in front of the current vehicle, false
    //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
    
    bool too_close_front = false;
    bool too_close_back = false;
    
    int prev_size = (int) rVehicle.previous_path_x.size();
    
    double car_s = this->s;
    if (prev_size > 0)
        car_s = this->end_path_s;
    
    map<int, vector<Vehicle>>::iterator it;
    for (it = predictions.begin(); it != predictions.end(); ++it){
        
        Vehicle temp_vehicle = it->second[0];  //WX: current TimeStep, see function below
        
        // car is in my lane
        if (temp_vehicle.lane == lane){
            
            double vx = temp_vehicle.x_dot;
            double vy = temp_vehicle.y_dot;
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = temp_vehicle.s;
            
            // if using previous points can project s value out
            // WX: this is for the other car !!! assume the same horizon as the ego
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

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> &predictions) {
  
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
            if (!too_close_right[0] && !too_close_right[1])
                this->lane ++;
        }
        this->d = 4 * this->lane + 2;
        
        this->target_speed -= .224;
    }
    else if(this->target_speed < this->speed_limit){
        
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

vector<Vehicle> Vehicle::get_trajectory_from_JMT(vector<double> jmt, double T, bool bPrint){
    
    vector<Vehicle> traj;
    
    int pts_count = (int) T / this->time_per_timestep;
    double t = 0;
    for (int i = 0; i < pts_count; ++i){
        
        t= this->time_per_timestep * i;
        double s = jmt[0] + jmt[1] * t + jmt[2] * t*t + jmt[3] * t*t*t + jmt[4] * t*t*t*t + jmt[5] * t*t*t*t*t;
        
          //debug
          std::cout << s << ", ";
      
        vector<double> XY = getXY(s, this->d, *this->p_mapPts_s, *this->p_mapPts_x, *this->p_mapPts_y);
        
        vector<double> car_info = {0, XY[0], XY[1], 0, 0, 0, 0};
        traj.push_back(Vehicle(car_info));
    }
    
    //debug
    std::cout << std::endl;
    
    return traj;
}

// ********** WX newly defined method: End ********************

vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> &predictions) {
  
    
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
