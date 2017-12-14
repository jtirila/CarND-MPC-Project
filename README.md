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
straight line. The biggest differences are the following. 

* The desired path, and hence also the polynomial fit, is not a straight line but a full lap around a lake. 
* To account for various phenomena resulting e.g. from the shape of the track, the coordinates are handled a bit 
  differently: at each time step, the origin of the 2D coordinate plane is at the location of the car, 
  the positive x axis points in the direction where the car is headed and the positive y axis to the left of the car 
  * This, together with a suitable window of interest, is eanough to guarantee that a polynomial modeling y as a 
    function of x is not singular (infinite slope or a vertically overlapping path 

* **State:** As hinted in the instructions, I ended up using always `0, 0, 0` as the reference state 
  and converting everything else (waypoints etc) into the corresponding coordinate system. 
* **Steering:** Due to the simulator's conventions, the received steering angle needs to be normalized 
  by multiplying it with -0.4363 (25 degrees in radians, and -1 to swap directions)
* **Speed:** the simulator sends speed in mph and as we want to deal with metric distances (x, y, CTE etc.), the speed 
  needs to be converted into meters per second. This conversion is carried out as
    ```
      v *= 0.44704;
    ```
* **Throttle:** No conversion is carried out for the throttle even though I speculated probably 
  a throttle of 1 does not really correspond to an acceleration of `1 m/s^2`. I experimented with normalization  
  coefficients ranging from 1 to 4 but observed no useful effect. I beliefe the throttle in the simulator 
  does not correspond to a fixed acceleration rate but rather depends on speed in the very least but 
  I have not confirmed this hypothesis. 

#### Update equations

While the update equations are largely similar to those int the line quiz, there are some differences: 
* The CTE calculation seems to me to be wrong in the material. I implemented it as: 
  ```
    cte[t+1] = y[t] - f(x[t]) + v[t] * sin(epsi[t]) * dt
  ``` 
* Inclusion of f and psides: in the process, I had suspicions that these need to be 
  included at each time step in the "augmented state vector". After finding out 
  that the values were remaining the same throughout, I made some changes and verified 
  the desired behavior. Left them in for debugging purposes. The equations, along with the other remaining update equations, are: 
  ```
    x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
    y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
    psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
    v_[t+1] = v[t] + a[t] * dt
    f[t+1] = polyeval(coeffs, x0 + v0 * CppAD::cos(psi0) * dt)
    psides[t+1] = CppAD::atan(coeffs[1] * 2 + coeffs[2] * x0 + 3 * coeffs[3] * x0 * x0)
    epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[1 + f_start + t] =
          f1 - polyeval2(coeffs, x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + psides_start + t] =
          psides1 - CppAD::atan(coeffs[1] * 2 * coeffs[2] * x0 + 3 * coeffs[2] * x0 * x0);
  ```
  Here, the parameter to atan is the derivative computation of the polynomial. I also experimented
  with automatic differentiation and poylynomial evaluation using the Poly function of CppAD, but was not able to figure out how to make it 
  work correctly. 


### Dealing with Latency

My approach to dealing with the 100ms delay before actuations have an effect was as follows. 
  1. Predict the vehicle's location in 100ms using the location, angle, speed and steering information 
     from the previous step as the parameters. The functions to perform this prediction are listed below.  
     
        ```
          double pred_dt = 0.1;
          double delta_x = v * pred_dt * CppAD::cos(psi);
          double delta_y = v * pred_dt * CppAD::sin(psi + v / 2 * Lf * steer * pred_dt);
          ...
          px += delta_x;
          py += delta_y;
        ```
 
     The inclusion of steering requires an explanation. I wanted to incorporate the effect of steering in to the 
     prediction, but only in a rough manner. This is not an exact solution, but a rough attempt at arriving at a 
     ballpark vicinity of the correct x, y point. I speculated that even an exact calculation would have been possible as it 
     is essentially just an integral of the sin function over time, but didn't bother with this as the 
     approximation seemed to yield good enough results. The rationale behind using half the full turn is to pretend as though 
     the car had been heading to an "average changed direction" instead of starting off with an angle of 0 and then 
     slowly proceeding to turn into the direction of the steering angle. 
  2. After the optimization procedure, choose the very first actuator values that correspond to the car's prediction 
     position after the latency delay.

### Choice of N and dt

My main points when considering `N` and `dt` were: 
* Not too many points because the track is rather smooth and only a few are needed for a good polynomial fit
* `dt` so that the timespan of the optimization step covers a reasonable time range into the future, 
  "around the next corner" so that we don't end up too deep in a micro-local optimum. 
  
With this reasoning, and some trial and error, I ended up choosing `N = 5` and `dt = 0.4`.

### Polynomial fitting and waypoint preprocessing 

My polynomial fit was adapted to take the vehicle's predicted location into account, so that all the 
waypoint coordinates are as seen from the predicted car coordinates and viewing angle. As the viewpoints are provided 
in the map coordinate system, they need to be transformed into the coordinate system of the vehicle. This 
is again an affine transform, implemented with the following piece of code: 
```
std::vector<double> MapCarTransform(double mx,
                                    double my,
                                    const std::vector<double>& car_coords){
  double theta = -car_coords[2];

  double cx = car_coords[0];
  double cy = car_coords[1];

  std::vector<double> result{
      std::cos(theta) * (mx - cx) - std::sin(theta) * (my - cy),
      std::sin(theta) * (mx - cx) + std::cos(theta) * (my - cy)
  };
  return result;
}
```
The polynomial fit is then performed for this set of points.


### The optimization: Tuning the Cost Function

As per the lectures, I added some coefficients to the terms in the cost function to give some 
phenomena more weight. The result can be seen in the code and results from a combination of intuitive 
reasoning and trial and error. 

## The result

To see how the code performs, you can run the code, or alternatively just sit back and watch one lap of driving in the 
Udacity simulator (click the image to view): 

[![D Control](https://img.youtube.com/vi/yuPbixOfkO4/0.jpg)](https://www.youtube.com/watch?v=yuPbixOfkO4)

## References

In addition to the documentations of the included libraries, and some StackOverflow answers, I skimmed through the 
related Udacity discussion board. 
