# Extended Kalman Filter Project Starter Code

Project for the Self-Driving Car Engineer Nanodegree Program

---

## Overview

In this project I implemented an Extended Kalman Filter algorithm in C++ capable of tracking a pedestrian's motion in two dimensions.

## Dependencies

* cmake >= 3.5
    * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
    * Linux: make is installed by default on most Linux distros
    * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
    * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
    * Linux: gcc / g++ is installed by default on most Linux distros
    * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
    * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`


## Files in the src Folder

* `main.cpp` - communicates with the Term 2 Simulator receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE
* `FusionEKF.cpp` - initializes the filter, calls the predict function, calls the update function
* `kalman_filter.cpp` - defines the predict function, the update function for lidar, and the update function for radar
* `tools.cpp` - function to calculate RMSE and the Jacobian matrix

The only files I modified are `FusionEKF.cpp`, `kalman_filter.cpp`, and `tools.cpp` where TODOs appear.

## How the Files Relate to Each Other

Here is a brief overview of what happens when you run the code files:

1. `Main.cpp` reads in the data and sends a sensor measurement to FusionEKF.cpp
2. `FusionEKF.cpp` takes the sensor data and initializes variables and updates variables. The Kalman filter equations are not in this file. `FusionEKF.cpp` has a variable called ekf_, which is an instance of a KalmanFilter class. The `ekf_` will hold the matrix and vector values. You will also use the `ekf_` instance to call the predict and update equations.
3. The `KalmanFilter` class is defined in `kalman_filter.cpp` and `kalman_filter.h`. I only modified `kalman_filter.cpp`, which contains functions for the prediction and update steps.

