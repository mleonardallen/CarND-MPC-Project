# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Project Video

[![Project Video](https://github.com/mleonardallen/CarND-MPC-Project/blob/master/images/capture.png)](https://youtu.be/qXTTggeUICk)

## Model

I used a simple kinematics model for this project, ignoring dynamics such as tire forces, gravity, and mass.  While a dynamics model accounts for more factors, the kinematics model is sufficient for this project.

### Variables

| Kinematic Variables | Meaning |
| -- | ------------- |
| x | x position |
| y | y position |
| ψ | orientation angle |
| v | velocity |
| eψ  | error from desired orientation |
| cte | cross track error |
| Lf | distance between the front of the vehicle and its center of gravity |

### Update Equations

![Kinematics Equations](https://github.com/mleonardallen/CarND-MPC-Project/blob/master/images/kinematics.png)

| Actuators | Meaning |
| --------- | ------- |
| δ (delta) | steering angle |
| a | acceleration [-1, 1], where negative values indicate braking. |

The purpose of the MPC is to optimize these actuators in order to minimize error in following the desired trajectory.

### Cross Track Error

The difference between the desired track and the vehicle position.

![Cross Track Error](https://github.com/mleonardallen/CarND-MPC-Project/blob/master/images/cte.png)

### Orientation Error

The difference between the desired orientation and the vehicle orientation.

![Orientation Error](https://github.com/mleonardallen/CarND-MPC-Project/blob/master/images/epsi.png)

## Timestep Length and Elapsed Duration (N & dt)

I found that the predicted distance covered `N * dt` should be approximately the same as given waypoints, but should not exceed the total length of the given waypoints.  When exceeding the waypoint distance, If found the MPC tends to overfit and provide erratic trajectories.  In contrast if the predicted distance making the car swerve a lot in response to the changing environment.

The predicted distance is also dependent on the velocity of the vehicle.  For instance N * dt at `v = 60` covers more distance than at `v = 40`.  At higher speeds, dt is should be reduced since the car is covering more distance and therefore will need to more quickly adjust position.

### Trial Values 

| N | dt | Result |
| - | -- | ------ |
| 5 | 0.05 | Vehicle jitters back and forth eventually driving erratically. |
| 10 | 0.05 | Vehicle jitters back and forth eventually driving erratically. |
| 15 | 0.05 | Vehicle jitters back and forth eventually driving erratically. |
| 5 | 0.1 | Model underfits; vehicle veers off track. |
| 10 | 0.1 | Model performs very well. |
| 15 | 0.1 | Model overfits; vehicle mostly stays on track but swerves in places. |
| 5 | 0.15 | Model underfits; vehicle veers off track. |
| 10 | 0.15 | Vehicle mostly stays on track but swerves in places. |
| 15 | 0.15 | Model overfits; vehicle mostly stays on track but swerves in places and touches yellow line. |

Further tuning could potentially lead to good model results for many of these trial values.  For example perhaps the error weights just need adjusted.

## Polynomial Fitting and MPC Preprocessing

Before fitting a polynomial to the desired waypoints, I first translate and transorm the waypoints to the car's perspective by applying a [Rotation Matrix](https://en.wikipedia.org/wiki/Rotation_matrix), and a 3rd order polynomial is then fit against the transformed waypoints.

With the polynomial coefficients, the initial state is calculated to be handed off along with the polynomial coefficients to the MPC procedure.

| Initial State (without latency considered) | Description |
| ------------- | ----------- |
| x | `0` due to transform |
| y | `0` due to transform | 
| ψ | `0` due to transform |
| v | velocity reported by server | 
| cte | cross track error calculated by evaluating polynomial at `x = 0` |
| eψ | the difference between the desired orientation and the vehicle orientation at `x = 0` |

## Model Predictive Control with Latency

Latency is the delay between sending a steering command and when vehicle actuators perform the command.  To account for latency, I predict the state of the vehicle after the latency period.  This is used as the initial state for the MPC procedure instead of the current state.  For example, the kinematics equations are used to determine the new initial `x position` with `dt = latency`.  See latency code: https://github.com/mleonardallen/CarND-MPC-Project/blob/master/src/main.cpp#L127-L132

![x at t+1](https://github.com/mleonardallen/CarND-MPC-Project/blob/master/images/xt+1.png)

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
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
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).
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
