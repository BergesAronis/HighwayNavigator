# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Reflection
The highway navigator is an implementation of a path finding algorithm to allow a vehicle to safely navigate a highway. This program has 2 key features, the first being the car maintains a safe distance from vehicles in its lane and the second being the ability to safely pass cars in its lane when this is an optimal movement.

This model can use the given sensor data to determine if a vehicle is in its lane and blocking it from moving forward or if there are vehicles to the left or right of it. The model uses this to decide if the car can and should change lanes to move ahead. While maintaining a safe distance from the vehicle in front of it, it will use the sensor data to determine if there is another path available.

To keep trajectory smooth, the model implements a 50 point path system. This means the trajectory is always made up of 50 waypoints and when any are removed, new points are appended to create a new 50 point path. The trajectory itself is calculated using the spline class, we use the spline to fit the cars actual movement to the pathed trajectory. 


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.


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
