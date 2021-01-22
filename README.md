# **PID Control** 

## Udacity Project 8 Writeup

---

**PID Control Project**

The goals of this project are the following:
* implement car steering using PID
* car must be able to drive the entire simulator track lap without going off track 
* experiment with PID coefficients and explain their meaning
* experiment with PID coefficients tuning 


---

In comparison to course material the key code modifications / customizations are in "PID.cpp" and related to:
* implementing the PID controller
* implementing the Twiddle algorithm

**Summary:** For this project I implemented the basic PID controller for vehicle steering and adjusted the weights manually (see description below). Following that I implemented Twiddle and experimented with various parameter settings and different initializations - I describe those experiments below, however I feel the addition of Twiddle algorithm did not give significantly better results over manual tuning, therefore my final solution that achieved the project goal is with manually tuned weights.


## PID controller coefficient experiments

I did the coefficient experiments in similar order as in the course lessens, starting with P, then moving to PD, ending up with PID:
* **Start with 0.0 ; 0.0 ; 0.0** , car just moves straight not adjusting the steering
* **Modify the P coefficient:** increasing this value causes the car to oscillate around lane center. When the car gets too far from center P controller makes a big correction towards the other side, eventually overshooting, then it makes another bigger correction towards the other side etc. This ends up overshooting more and more, causing car to get out of the lane further and further after each correction.
* **Modifying D coefficient:**  compensates for this overshooting of the P controller. If the value of D is too small the overshooting starts after longer distance.
* **Modifying the "I" coefficient** is supposed to resolve the issue of a systemic bias (e.g. the steering drift as in the lessons or potentially some added error due to car/road condition). In case of simulator it doesn’t seem to have much effect. If the value of "I" is set to zero the car can already drive entire route, making me believe there isn't much systematic bias in this case. Increasing the "I" value too high causes the car to compensate too much and get off the lane similar as when there was no D coefficient earlier on. 

### Tuning the coefficients

* In line with described effects of modifying PID coefficients, I started with 0;0;0 and modified one by one setting values similar to the one in course lessons (i.e. 0.2; 0.04; 3.0)
* After setting p =0.2 and d=3.0 the car could already drive the entire track. The i= 0.04 turned out too big, lowering it by an order of magnitude to 0.004 which fixed to issue but since I = 0.0 also works I kept it that way.


## Additional experiments

* I implemented the Twiddle algorithm to see how it would work in the simulator environment. However, after initializing it with my hand picked coefficients it didn't seem to modify them much. Otherwise, if I init with "bad" coefficients, e.g. all zeroes like in the course lessons, Twiddle doesn't tweak them sufficiently and car gets out of the road. Likewise, the initial values matter for the delta values. Too big (e.g. like in the class, all equal to 1) make the car to quickly get off the road, too small don’t seem to make any meaningful impact. I experimented with initial values of delta equal to /10 and /100 of the coefficients - in the end they don’t seem to lead to big changes in the coefficients. I also experimented with turning off tolerance and let the algorithm run all the time; and with different ways of calculating the avg error. Perhaps with more experiments Twiddle could be made to work as expected but I didn’t continue as for the project rubric it was sufficient with manual adjustment. I left the Twiddle algorithm in the code to document my experiments, however it could be commented out since doesn’t make much difference.
    * /10 - settles with Kp: 0.312032 Kd: 4.22878 Ki: 0.000532 [starting point : 0.2, 0.0004, 3.0]
    * /100 - settles with  Kp: 0.211203 Kd: 3.13923 Ki: 0.000423313 [starting point : 0.2, 0.0004, 3.0]
* I also did few experiments with PID for throttle but also non-conclusive. The avg error calculation seemed to have big impact (squared not working particularly well, abs better), however in no configuration did I manage to achieve effects better than with constant throttle (safety wise), so I left my final implementation with the PID for throttle.

## Project Execution Info (by Udacity) 

---

### Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

### Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

### Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

### Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

### Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

### Call for IDE Profiles Pull Requests

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

### How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

