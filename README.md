# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program


## PID controller

This project implement a PID controller that steers a car in a simulator.
A basic PID controller determines the control value u(t) by:

  u(t) = Kp * e(t) + Ki * Integral( e(t) ) + Kd * de(t) / dt

where Kp, Ki, and Kd are tuning parameters and e(t) is the current error between the setpoint value and the measured value.

![alt text](https://upload.wikimedia.org/wikipedia/commons/thumb/4/43/PID_en.svg/400px-PID_en.svg.png)

The proportional term is proportional to the current error. 
The integral term is proportional to both the magnitude of the error and the duration of the error. 
The derivative term is proportional to the slope of the error over time. 



## Parameter tuning

In order to obtain good parameters that result in keeping the car on a track. I made an initial
guess by playing around with the parameters individually.

For further optimization, a tuning algorithm is implemented. Therefore, I defined an error that measures how good/how bad the controller is performing:

  err = sum( e(t) ^ 2 ) )/ tuning_time ^ 1.5

where tuning_time is the time since the simulation started. Diving by the tuning time normalizes the sum of the squared error. Additionally longer driving is rewarded by dividing with tuning_time ^ 1.5.

The tuning algorithm is triggered when either

* cte > 2.5
* tuning_time > 45 s

For automatic parameter tuning the tuning algorithm 'twiddle' is used.
By altering each parameter individually and checking if an overall error is reduced the algorithms
finds better parameters. 

I ended up with Kp = 0.27528, Ki = 0.19602, Kd = 0.121
