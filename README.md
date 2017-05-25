# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Reflection

### Vehicle Model

```
State:
 - x   : position in the forward direction relative to the vehicle
 - y   : position in the lateral direction relative to the vehicle
 - psi : vehicle orientation
 - v   : velocity

Actuators:
 - delta : steering angle in radians
 - a     : acceleration

Other:
 - dt : timestep length
 - Lf : distance from the center of the vehicle to its front

Next state:
 - x_t1   = x_t + v_t * cos(psi_t) * dt
 - y_t1   = y_t + v_t * sin(psi_t) * dt
 - psi_t1 = psi_t * v_t / Lf * delta * dt
 - v_t1   = v_t + a_t * dt
```

We are using vehicle dynamic model mentioned in the lecture. The x axis is
pointing at the vehicle's moving direction at time t. The x value loaded from the simulator is on a fixed coordinate system, so a coordinate transform is required. All trajectory points should be mapped to the new vehicle coordinate system.

```
new_x = (ori_x - vehicle_x) * cos(-psi) - (ori_y - vehicle_y) * sin(-psi)

new_y = (ori_x - vehicle_x) * sin(-psi) + (ori_y - vehicle_y) * sin(-psi)
```


### Timestep Length and Frequency

```
Frequency (dt): 0.1
Timestep length (N): 8
```

After several attempts, I notice that dt * N controls the length of the green line in the simulator. If the line is too short, the vehicle couldn't react to the turn well. It couldn't make enough steering angle. When the line is too long, it tends to draw funny lines. The best range is around 8 to 10.

I was trying 0.05 for dt to get enough points and it worked well. In order to reduce the calculation, I start increasing dt. The maximum is 0.1 because that is the amount of latency. It is easier to handle latency when we use a timestep with the length that can represent the latency.


### Polynomial Fitting and MPC Preprocessing

```
Polynomial form: ax^2 + bx + c

Cost function: 10 * cte^2 + epsi^2 + ev^2 + 1000 * delta^2 + a^2 + d_delta^2 + d_a^2
```
I tried first, second, and third degree polynomails. The first degree polynomial represents a line. It can't capture the trajectory well. Second and third degree seem to be doing well. Since I can't tell the difference, I chose the second degree polynomial with less calculation.

The cost function takes cte, yaw error, velocity error, acceleration, delta, difference of deltas, and difference of accelerations into account. Since steering angle is the most importance factor we care, I give it a larger weight. And it seems working without adding weights to other terms. Also, we need larger weights when the speed is higer.


### Model Predictive Control with Latency

When the latency is applied, the vehicle oscillates around the trajectory. It gets worse with higher speed. In order to compansate the delay, we need to return the predicted steering angle ahead in the future. I can simply be done by returning the second value from ipopt result. We also can smooth the value by averaging values around it.


### Steering angle and throttle

In the simulator, a positive angle means to steer right. However, in the vehicle dynamic model, a positive delta indicates turning left, so we have to apply negative scaled delta value to steering angle.

The lower bound and upper bound of acceleration is -1 and 1, which is the same as the range required by the throttle. We can simply assign `a` to throttle.


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
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt --with-openblas`
   * Some people have reported `--with-openblas` causes issues. If this is the case install without it `brew install ipopt`.
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
