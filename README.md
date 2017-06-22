# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Introduction

Kinematic models and control systems are crucial for the autonomous vehicle in order to safely navigate through a lane. The purpose of this project is to improve the previous PID controller by designing and implementing a Model Predictive Controller (MPC) that guides the car in a simulator. The vehicle physics have been modelled using the so-called bicycle kinematic model, which is non-linear but simple enough. The simulation has been programed in C++.

## Kinematic model
The model used is the so-called bicycle kinematic model, which considers (1) the position of the vehicle (x and y coordinates), (2) the orientation (psi), and (3) its velocity (v).  Also, the model considers two actuators: (1) the steering wheel (delta), and (2) the throttle pedal (a). Note that the throttle pedal is used for both braking (negative values) and for accelerating (positive values). The model uses the below equations.

![Figure 1: Kinematic model. Image taken from the course.](https://github.com/andreuSancho/CarND-MPC-Project/blob/master/equationsLarge.png)

The above equations have been taken from the course.

## Length of the trajectory (N) and elapsed duration (dt)
The first step in the process is to define the length of the trajectory (N) and the elapsed duration of each time step (dt) hyperparameters. These have been found empirically and are set as follows:

-	N = 11
-	dt = 0.1

These values are defined in the `constants.h`, lines `6` and `7`.  The prediction horizon is, therefore, 1.1 seconds. Other values were explored in a grid search-alike way: `(10, 0.1)`, `(15, 0.1)`, `(17, 0.1)`, `(18, 0.1)`, `(19, 0.1)`, `(20, 0.1)`, `(30, 0.1)`, `(20, 0.05)`, and `(30, 0.05)`. However, some of these produced oscillations in the trajectory and have been considered unsafe.

## Polynomial Fitting and MPC Preprocessing
As recommended by the instructors, provided waypoints are preprocessed by transforming them into vehicle coordinates. The transformation is nothing more than a translation and a rotation. This way the state vector to be passed to the solver has x, y and psi parameters set to zero, requiring much less computations during the fitting (see lines `103` to `120` in the file `main.cpp`).

## Latency
A complex part, but critical for successfully driving the car with the MPC, is to take the latency into the model. For doing so, three distinct aspects where used: (1) adding a `sleep` command of 100ms just before submitting the new commands to the simulator (see line `159` in the file `main.cpp`), (2) taking the actuators from a delayed time step (see lines `82` to `85` in the file `MPC.cpp`, and (3) adding an extra penalty in the cost function, which penalizes the sum of the velocity and steering wheel angle (see line `48` in the file `MPC.cpp`). Also, the speed has been limited to 80 mph (see line `19` of the file `constants.h`).

Note that the last to solutions were strongly inspired by the work of Jeremy Shannon (https://github.com/jeremy-shannon).

## Result

Video of the car controller driving in the training (click to play).
[![MPC controller in action](https://img.youtube.com/vi/cPGv7XdXPpI/0.jpg)](https://youtu.be/cPGv7XdXPpI)

## Note on using the code in Windows.

Windows users have to do a bridge between the Virtual Machine (VM) with Linux and the Windows Host. To do so, open the console as administrator and type the following:

`netsh interface portproxy add v4tov4 listenport=4567 listenaddress=127.0.0.1 connectport=4567 connectaddress=<LINUX_IP> protocol=tcp`

where *<LINUX_IP>* is the IP address of the Linux VM machine.

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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

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

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
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
"# CarND-MPC-Project" 
