# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Kemal Tepe, ketepe@gmail.com

### Objective: 

To design a path planning algorithm for autonomous driving using finite state machines.

### Project Description:

In this project, a pathfinding algorithm is implemented to control the car's driving behavior steer around the track in a most favorable path without an accident. The algorithm consists of several parts: (1) control the car's movement in a lane without exceeding the required acceleration and deceleration limits, and maximum speed and keeping the car not colliding with a car moving in front of it, if there is a car in front of our car, (2) identifying which lane is the best option for this car to do continue its journey, and (3) steer the car such that without causing an accident. Once the car is in the new lane, then continue the process. 

Previous lectures on path planning were useful in the implementation of the project. Also, the walkthrough session was done by David Silver, and Aaron Brown was instrumental to get us started. Also, I enhanced some aspects of the walkthrough session in my implementation.

#### (1) Control of the movement in the lane.
The car continuously monitors the sensor_fusion outputs to identify if there is a car in front of it. The section of the code which performs this part of the algorithm is given below but the most critical section is if the front car is within 30m of our range, then we identify if the car is too_close and move to lane_change section of the code. In identifying the car in front of us, I only used the distance between the vehicles and this was sufficient in this simple simulation case, but the speed of the vehicle in front could also be utilized to better judge our actions. For example, if the front car is faster or at similar speed there would not be a need to move to the lane_change section.

```C++
for (int i=0; i< sensor_fusion.size(); i++)
            {
                float check_car_d = sensor_fusion[i][6];                    
                int check_car_lane=db.identify_lane(check_car_d);
                
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
//                double check_car_speed = sqrt(vx*vx+vy*vy);
                double check_car_s = sensor_fusion[i][5];
    
                //check_car_s+=((double) prev_size*0.02*check_speed);
                //if ith car is in our lane
                if (car_lane == check_car_lane )
                {        
                    if ((check_car_s > car_s) && ((check_car_s-car_s) <30))
                    {    // need to slow down and look for a change of lane opportunity
                        too_close = true;
                        car_lane_reward=db.lane_reward(car_s, check_car_s, 0, 0, true);
                        check_lane_reward[car_lane]= car_lane_reward;
#if ket_debug
                        cout << check_car_s << " " << car_s << "  too close "<< endl;
                        cout<< "current_lane " << car_lane << " cur_lane_reward "<< car_lane_reward << endl;

#endif                                        
                        goto lane_change;
                        
                    }
                }        
            }
            lane_change:

```

#### (2) Lane change.

Once the car identifies it is too_close to the front car, the algorithm jumps to this section of the algorithm. The first task here is to identify if we can really change the lane. In order to do this, we need to calculate the cost (reward) of changing lane. I do this calculation using the following method in ```DrivingBehcave``` class in ```ket.h``` file. The method is as follows:

```C++
    
double DrivingBehave::lane_reward(double car_s, double check_car_s, double car_speed, double  check_car_speed, bool ahead)
{ 
    double reward = 0.0;
    double speed_bias=0.05;
    double distance_bias=1.0;
    if(ahead) {
        // distance^2+speed_dif^3
        // if ahead car is faster better,
        reward = distance_bias*(check_car_s-car_s)*(check_car_s-car_s);
        //did not include the speed for this version
        //reward+= speed_bias*(check_car_speed-car_speed)*(check_car_speed-car_speed)*(check_car_speed-car_speed);
        return reward;
    } 
    else
    { //distance^2+speed_dif^3
        // if behind car is slower is better
        reward = distance_bias*(car_s-check_car_s)*(car_s-check_car_s);
        //reward += speed_bias*(car_speed-check_car_speed)*(car_speed-check_car_speed)*(car_speed-check_car_speed);
        return reward;
    }    
}
```

I although in the routine I included speed of the other vehicles in the reward, meaning that if the car in front is moving faster as well as ahead, this lane should be more advantageous. But in order to make the system simpler, I only used distance-based cost (reward) value.

Then, these rewards are calculated for each vehicle in sensor_fusion. Since I used distance-based metric, it did not matter is the car in the next lanes are behind or front. However, we can also include speed of next lane vehicles in the reward. For example, if a car is faster than us and coming behind, it should be less advantageous to switch to a lane where a car is a slower or equal speed than us. Once the rewards calculated from the distance to each car in the sensor_fusion, then the minimum of these rewards is assigned as the reward for each lane as given by the following lines: 
 
```C++

    DrivingBehave db;
    double car_lane_reward=10000.0;
    int car_lane=db.identify_lane(car_d);
    vector<double> check_lane_reward;
    for (int i=0; i< 3; i++) check_lane_reward.push_back(10000.0);
    
    for (int i=0; i< sensor_fusion.size(); i++)
    {
        float check_car_d = sensor_fusion[i][6];
        int check_car_lane = db.identify_lane(check_car_d);
        double check_car_s = sensor_fusion[i][5];
        
        if (check_car_lane!=car_lane)
        {
            double rew=db.lane_reward(car_s, check_car_s, 0, 0, 1);
                
            if ( rew < check_lane_reward[check_car_lane]) 
                check_lane_reward[check_car_lane] = rew;
            }    
                
        }
    }
```
Once lane rewards are calculated, then by using the following logic, we identify if we need to keep the existing lane or switch to another lane as done by

```C++
    lane_change:
            //lane change 2->1;                
#if ket_debug
            //for (int i=0; i< 3; i++) 
                cout << "lanes 0-1-2: " << check_lane_reward[0] << " " << check_lane_reward[1] << " " << check_lane_reward[2] << " " << car_lane_reward << endl;
#endif

            double lane_bias=2;
            double min_reward=400;
            if (car_lane == 2)
            { 
                if (check_lane_reward[1] > car_lane_reward*lane_bias and check_lane_reward[1] >=min_reward) 
                    lane=1;
            }
            //lane change 0->1;
            
            if (car_lane == 0)
            {
                if (check_lane_reward[0] > car_lane_reward*lane_bias and check_lane_reward[0] >=min_reward) 
                    lane =1;
            }            
            //lane change 1->2 or 1->0 possible;
            if (car_lane ==1)
            {
                if ((check_lane_reward[0] > check_lane_reward[2]) and (check_lane_reward[0] > car_lane_reward*lane_bias) and (check_lane_reward[1] >=min_reward)) 
                    lane = 0;
            }
            if (car_lane ==1)
            {
                if ((check_lane_reward[2] > check_lane_reward[0]) and (check_lane_reward[2] > car_lane_reward*lane_bias) and (check_lane_reward[1] >=min_reward)) 
                    lane = 2;
            }
            //reduce speed

            if(too_close)
            {
                ref_vel -=0.224;
            } else if (ref_vel < max_velo)
            {
                ref_vel +=0.224;
            }
            

```

Above, I limited lane changes only to next one at a time. To change a lane the lane reward of the new lane should be slightly higher to avoid oscillation between lanes, as well as to reinforce the idea to have a good reason to have a lane change. Here I used variable ```lane_bias``` to adjust this. Values between 1.2 and 2 provided the good compromise. Also if the reward is low, even it is more than our own lane, we should keep our lane and just adjust our speed. Then we adjust our speed again to make it slower or faster. 



(3) Identify the next path based on the lane change or no lane change.

One the new lane (or keep the old lane) decision made and speed are adjusted, then the next speed is to identify waypoint, then using the spline provides a path which provides a smooth transition to this new path. The following code is used to do this in the algorithm. The new path are identified first in car coordinate system, then transferred to global (map) coordinate system. Then spline function identifies the smooth curve for the car to follow in the simulation environment. This part is mostly redoing the walkthrough lecture's code. Aaron has more comprehensive explanation of this section.


```C++
vector<double> next_wp0 = getXY(car_s+wp_step_size, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s+2.0*wp_step_size, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s+3.0*wp_step_size, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            
ptsx.push_back(next_wp0[0]);
ptsx.push_back(next_wp1[0]);
ptsx.push_back(next_wp2[0]);

ptsy.push_back(next_wp0[1]);
ptsy.push_back(next_wp1[1]);
ptsy.push_back(next_wp2[1]);
            
for (int i=0; i < ptsx.size(); i++)
{
// shift car ref angle to 0 deg.
double shift_x = ptsx[i] - ref_x;
double shift_y = ptsy[i] - ref_y;
            
ptsx[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
ptsy[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
}
            
tk::spline s;
    
//set(x,y) points to the spline
s.set_points(ptsx, ptsy);
```

### Conclusions:

* A pathfinding algorithm based on a finite state machine is implemented. In this algorithm, the cost metric and lane change operation were kept simple enough to satisfy the project requirements. A more elaborative cost function which includes other vehicle's position and speed would be necessary for real-life scenarios. 

* Also, I did not implement circular nature of the track in my implementation. At the transition where car_s is close to max_s, and other vehicles are s values are greater than max_s (then between [0, max-s]) there may be problems, but in my runs, I did not notice any problem. 

* Overall the project was interesting and very educational, as well as challenging. 


---Remaining sections are verbatim copy of the Project's original Readme file----

# CarND-Path-Planning-Project (original readme file)
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

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

