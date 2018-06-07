# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Rubric
#### Compilation
The code compiles with `cmake` and `make` without errors.

#### Valid Trajectories
* The car is able to drive at least 4.32 miles without incident.
* The car drives according to the speed limit.
* Max Acceleration and Jerk are not Exceeded.
* Car does not have collisions.
* The car stays in its lane, except for the time between changing lanes.
* The car is able to change lanes smoothly.

![pathplanning](https://github.com/nikhil-sinnarkar/CarND-Path-Planning-Project-master/blob/master/pathplanning.jpg)

#### Reflection
In this project we need to define a path made up of (x,y) points that the car will visit sequentially every .02 seconds.
The simulator returns us the previous path which the car was following. This may be empty if we have just started or the car has fully executed the path. In case it is not empty we use this to build our path in continuation of the previous path in order to avoid sudden changes in our heading or speed. This is implemented in [main.cpp](https://github.com/nikhil-sinnarkar/CarND-Path-Planning-Project-master/blob/master/src/main.cpp) from line 388 to 418.

After this we add evenly 30m spaced points ahead of the starting reference in frenet coordiante.
```
vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
```
We add these points to the list of x and y points. After this we transform the coordinates such that our car is at (0,0) with a heading of 0 degrees. 
```
for (int i = 0; i < ptsx.size(); i++)
{
	  //shift the c ar reference angle to 0 degrees
	  double shift_x = ptsx[i] - ref_x;
	  double shift_y = ptsy[i] - ref_y;
	  ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
	  ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
}
```
Then we fit a spline onto these points and convert the points that are lying on the spline as the points the car should follow.
We create an array of x and y points that we will pass to the planner. We append the points from our previous path to it and then fill the remaining array with points from the spline so it has a total of 50 points.

#### Lane change logic
First we scan through list of all the cars picked up by our sensors. In this list we first check if the car is in front of us (within 30 meters). If it is then we set a flag, if not then we proceed further. If the car is not in our lane then we check if it is in our right lane or left lane within a specific range. If the car is in our right or left lane and it is either 30m ahead of us or 10m behind then we should not move to that lane. Hence if this condition is detected I set the flag `car_in_left` or `car_in_right` to `true`. Also I calculate the speed of the cars which are in front of us either in our lane or right/left lane. After we have iterated through all the cars in the list we have the information if there is a car in front of us or in our left/right lane and close to us.
```
//car is in my lane
if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2))
{
	check_car_s += (double)(prev_size * 0.02 * check_speed); // if using previous points can project s value out
	// check s value greater than mine and s gap
	if ((check_car_s > car_s) && ((check_car_s - car_s) < 30))
	{
		too_close = true;
		front_car_speed = check_speed;
	}
}

//car in left lane
else if ((lane != 0) && (d < (2 + 4 * (lane - 1) + 2) && d > (2 + 4 * (lane - 1) - 2)))
{
	double curr_check_car_s = check_car_s;
	check_car_s += (double)(prev_size * 0.02 * check_speed);
	if ((check_car_s > car_s) && ((check_car_s - car_s) < 30))
	{
		car_in_left = true;
		left_car_speed = check_speed;
		// cout << "car detected on left front" << endl;
	}
	else if (abs(car_s - check_car_s) < 10)
	{
		car_in_left = true;
		// cout << "car detected on left" << endl;
	}
}

//car in right lane
else if ((lane != 2) && (d < (2 + 4 * (lane + 1) + 2) && d > (2 + 4 * (lane + 1) - 2)))
{
	double curr_check_car_s = check_car_s;
	check_car_s += (double)(prev_size * 0.02 * check_speed);
	if ((check_car_s > car_s) && ((check_car_s - car_s) < 30))
	{
		car_in_right = true;
		right_car_speed = check_speed;
		// cout << "car detected on right front" << endl;
	}
	else if (abs(car_s - check_car_s) < 10)
	{
		car_in_right = true;
		// cout << "car detected on right" << endl;
	}
}
```

Now we make the decision of changing the lanes based on the flags we have set and the speed of the cars in front of us. If we find a car in front of us then we check either left or right or both the lanes (depending on which lane are we driving currently) for free space. If we find a free lane then we make a turn to that lane. In case we are in center lane and both the right and left lanes sre free then left lane is preffered. 

Next case is that we have a car in front of us and a car in next lane also. In this case we first check the speed of both the cars and turn to the lane which has the car with higher speed. If the car in front of us has a higher speed then the car in the next lane then we won't change our lane.



---  
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

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

