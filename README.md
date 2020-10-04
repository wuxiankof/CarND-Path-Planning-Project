# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

## Reflection

### 1. Introduction

This Refection Section provides general descriptions of new classes/codes that I have created and the specific descriptions of the codes generating paths for the self-driving car.

### 2. Description of New Classes/Files I Have Created

1. **PathPlanner Class**

   - The PathPlanner Class is aimed to set up an environment taking the reference of all map way points, receiving location data of the self-driving car and fusion data of all other vehicles, and coordinate with all Vehicle objects at each time step;
   
   - Variables defined for this Class includes:
        - `Vehicle ego_Vehicle`: an object of the Vehicle class representing the self-driving car through out the entire simulation period.
        - `std::map<int, Vehicle> vehicles`: a map storing objects of the Vehicle class for all other vehicles in the simulation environment.
        - five vectors storing the map way points reference information.
        
   - Functions defined for this Class includes:
        - `void update_ego_vehicle(x, y, s, d, yaw, speed, previous_path_x, previous_path_y, end_path_s, end_path_d)`: to update the variable ego_Vehicle with the latest location/kinematics.
        - `void update_vehicles(sensor_fusion_data)`: to update all vehicle objects with the sensor fusion data stored in the map variable vehicles.
        - `void get_ego_trajectory(&next_x_vals, &next_y_vals)`: to obtain the planned trajectory which contains coordinates of points in the next 50 time steps.

2. **Vehicle Class**

   - The Vehicle Class is defined for both the self-driving car (ego vehicle) and all other vehicles in the simulation environment. It includes attributes and functions that trace vehicle kinematics, predict future trajectories and path planning for the ego vehicle.
   
   - Variables defined for this Class includes:
        - `double time_per_timestep`: time step of the simulation, which is set as 0.02 seconds per timestep as provided. 
        - `double v, v_prev, a`: current speed, speed in previous time step and current acceleration.
        - `double yaw`: current yaw value. If it's for the ego vehicle object, the unit is in degree/sec; otherwise, the unit is in rad/sec.
        - `float target_speed`: speed the vehicle is targeting to.
        - `float speed_limit`: the speed limit which is set to 49.5 MPH times 0.44704 (MPH converted to m/s: 1.6*1000/3600 or 1/2.24).
        - `string state`: state of the vehicle which is currently only set as the default state "CS" (Constant Speed). Will be expanded to other states in future (not in this submission).
         - other variables/pointers for the map way points reference information, as well as the previous path's information.
        
   - Functions defined for this Class includes:
        - `void Update_Vehicle_Info(car_info)`: to update vehicle info at each time step.
        - `float position_at(t)`: calculate position of the vehicle at time t.
        - `vector<Vehicles> generate_predictions(timesteps)`: generate predictions of paths for other vehicles.
        - `vector<bool> check_others(lane, &predictions, &rVehicle)`: check if there are adjacent vehicles (both front and back) in a perticular lane.
        - `vector<Vehicle> generate_trajectory(&predictions)`: generate trajectory.
        
3. **Others**
    
    - in the Helpers.h/.cpp, I have defined a convenient function `int d2LaneNumber(d)` which is to convert value d to the lane number. 
   
### 3. Specific Description of Codes for Generating Paths for the Self-driving Car

1. **At each time step (0.02 second)**:
   
   - the PathPlanner object will obtain all other vehicles' locations and pass it to the ego_Vehicle object.
   
   - then the ego_Vehicle object will make use of this information to carry out its path planning through the function `Vehicle::generate_trajectory(predictions)`.

2. **In the ego_Vehicle object's function** `Vehicle::generate_trajectory(predictions)`, it will plan the trajectory through the following steps:
   
   - Select starting point(s) of the trajectory. If the previous path is almost empty, the starting points will be the current location of the car together with one more point at its back; however, if the size of the previous path is greater than 2, the starting points will be the last two points of the path.
   
   - Check if there are other vehicles in the vicinity. The function `Vehicle::check_others(lane, vehicle)` will be used to check if there are other vehicle(s) in the vicinity. if there is a vehicle in front within 30 meters, the ego vehicle will slow down its speed by 0.224 m/s at each time step; otherwise, it will just increase the speed by the same acceleration rate. 0.224 m/s up to a speed limit 49.5 mph.
   
   - Try to perform lane changing if allowed. The function `Vehicle::check_others(lane, vehicle)` will also assist to check if there is sufficient space in adjacent lanes to perform lane changing. If there is, the ego vehicle will prepare to perform lane changing.
   
   - Smooth the path using spline. In order to meet the Rubrics requirements that the car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3, the path to be constructed need to be smoothed. The [spline Class](https://kluge.in-chemnitz.de/opensource/spline/) is adopted for this purpose. It includes the transformation of global coordinates to the vehicle's local coordinates to perform the spline interpolation, and then transform back again at the end.

3. **At the end of each time step (0.02 second)**, the code model will return a path trajectory which includes 50 points for the ego vehicle to execute next.