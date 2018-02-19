**Kidnapped Vehicle Project**


Self-Driving Car Engineer Nanodegree Program

This repository contains the code for the final project for the Localization course in Udacity's Self-Driving Car Nanodegree.

[//]: # (Image References)
[image1]: ./docs/img1.png
[image2]: ./docs/img2.png
[image3]: ./docs/img3.png


### Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data.

### Build

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

```
mkdir build
cd build
cmake ..
make
./particle_filter
```


## [Rubric](https://review.udacity.com/#!/rubrics/747/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Accuracy

#### Does your particle filter localize the vehicle to within the desired accuracy?
##### This criteria is checked automatically when you do ```./run.sh``` in the terminal. If the output says "Success! Your particle filter passed!" then it means you’ve met this criteria.

Here is a screenshot of simulator showing "Success! Your particle filter passed!" after doing one run.

![image3]



### Performance

#### Does your particle run within the specified time of 100 seconds?
##### This criteria is checked automatically when you do ```./run.sh``` in the terminal. If the output says "Success! Your particle filter passed!" then it means you’ve met this criteria..

Here is the same screenshot of simulator showing "Success! Your particle filter passed!", meaning the criteria was passed, in particular for this run in 53.52 seconds.

![image3]


### General

#### Does your code use a particle filter to localize the robot?
##### There may be ways to “beat” the automatic grader without actually implementing the full particle filter. You will meet this criteria if the methods you write in ```particle_filter.cpp``` behave as expected.

Particle filter was implemented in the files ```particle_filter.cpp``` and ```particle_filter.h``` of ```src``` directory, the rest of the source files were not changed.

Following methods where implemented as part of the particle filter:

* ```init```: Generates the initial random particles based on a simulated estimate from GPS.
* ```prediction```: Predicts the position of each one of the particles based on the increment of time, velocity and yaw rate.
* ```dataAssociation```: This one was modified from the original, now if finds which observations correspond to which landmarks using the nearest-neighbors data association.
* ```updateWeights```: Updates the weights for each particle based on the likelihood of the correspondence with the observed measurements.
* ```resample```: Resamples from the updated set of particles proportionally according with their weights.
* ```SetAssociations```: Just slightly modified to return the particle.

These are some screenshots from the simulator, showing the process in action:

![image1] ![image2] ![image3]

---

### Discussion

#### Briefly discuss any problems / issues you faced in your implementation of this project.

One of the problems I faced was the case when the yaw rate was zero or close to zero, since in the initial tests it generated non-arithmetic-numbers, during the prediction process which braked the process. The solution actually was very simple, just to check for the value of the yaw rate, and if it can be considered zero, the predicting process just involves the trigonometrical formulas based on theta, velocity and time increment.

Another point to consider was the selection of the number of particles to use. I was experimenting with several values 3, 10, 100, 250, 1000, 10000. The general tendency was that with higher values the error decreased, however the performance also dropped, p.e. between 100 and 10000 there was huge difference (around 110 for x and y for 100 particles, and around 100 for 10000), so in general the sacrifice of performance using huge numbers was not really worthy. The criteria could be successfully passed even with a sample of just 3 particles, however a value of 100 seemed to be good balance between performance and accuracy.
