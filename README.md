# CarND Model Predictive Control
*Self-Driving Car Engineer Nanodegree Program, Term 2, Project 5*

**By J-M Tirilä**

## Background

This repository contains my code for this project. For tech info (dependencies, running instructions etc.) refer 
to the [Udacity's repository for the project ](https://github.com/udacity/CarND-MPC-Project). 

## The task

The task in this project was to write code that implements the Model Predictive Control mechanism to 
drvive a vehicle around a track, based on a desired path received in the form of a collection of *waypoints* on each 
time step. 

## Some Notes Concerning my Solution

### Description of the model

The model implemented here corresponds pretty accurately to the lectures and the preceding quiz dealing with a 
straight line. 

* **State:** As hinted in the instructions, I ended up using always `0, 0, 0` as the reference state 
  and converting everything else (waypoints etc) into the corresponding coordinate system. 
* **Steering:** Due to the simulator's conventions, the received steering angle needs to be normalized 
  by * multiplying it with FIXME
* **Speed:** the simulator sends speed in mph and as we want to deal with metric distances (x, y, CTE etc.), the speed 
  needs to be converted into meters per second. This conversion is carried out as
    ```
      y = fx // FIXME
    ```
* **Throttle:** No conversion is carried out for the throttle even though I speculated probably 
  a throttle of 1 does not really correspond to an acceleration of `1 m/s^2`. I experimented with normalization  
  coefficients ranging from 1 to 4 but observed no useful effect. I beliefe the throttle in the simulator 
  does not correspond to a fixed acceleration rate but rather depends on speed in the very least but 
  I have not confirmed this hypothesis. 

#### Update equations

Explain the changes to the lecture version
* CTE calculation: wrong in lectures
* Inclusion of f and psides: in the process, I had suspicions that these need to be 
  included at each time step in the "augmented state vector". After finding out 
  that the values were remaining the same throughout, I made some changes and verified 
  the desired behavior. Left them in for debugging purposes. 


### Dealing with Latency

My approach to dealing with the 100ms delay before actuations have an effect was as follows. 
  1. Predict the vehicle's location in 100ms using the location, angle, speed and steering information 
     from the previous step as the parameters. The functions to perform this prediction are listed below.  
     
        ```
          y = fx // FIXME
          z = gx // FIXME TOO
        ```
 
     The FIXME requires an explanation. It is not an exact solution, but a rough attempt at taking the 
     effect of steering into account. I figured even an exact calculation would have been possible as it 
     is essentially just an integral of the sine function over time, but didn't bother with this as the 
     approximation seemed to yield good enough results. The rationale behind FIXME is to pretend as though 
     the car had been heading to an "average changed direction" instead of starting off with an angle of 0 and then 
     slowly proceeding to turn into the direction of the steering angle. 
  2. After the optimization procedure, choose the very first actuator values that correspond to the car's prediction 
     position after the latency delay.

### Choice of N and dt

My main points when considering `N` and `dt` were: 
* Not too many points because the track is rather smooth and only a few are needed for a good polynomial fit
* dt so that the timespan of the optimization step covers a reasonable time range into the future, 
  "around the next corner" so that we don't end up too deep in a micro-local optimum
  
With this reasonging, I ended up choosing `N = FIXME` and `dt = FIXME`.

### Polynomial fitting and waypoint preprocessing 

My polynomial fit was adapted to take the vehicle's predicted location into account, so that all the 
waypoint coordinates are as seen from the predicted car coordinates and viewing angle. As the viewpoints are provided 
in the map coordinate system, they need to be transformed into the coordinate system of the vehicle. This 
is again an affine transform, implemented with the following piece of code: 
```
  y = fx // FIXME
```
* Transform waypoints into a coordinate system whose origin is located at the car's predicted location, include code 
  snippet here
*  Fit polynomial to this set of points

### The optimization: Tuning the Cost Function

TBW


