# CarND-Controls-PID

In this project the following objectives are achieved:

- Implementation of PID controller in C++
- Setting the Hyper-parameter
- Using Twiddle to find the hyper-parameters

## Implementation of PID controller in C++

The PID controller was implemented using the help videos and the course. It uses the PID class `PID.ccp` in is done in two steps. The method `updateError` (Lines 25-30) calculates the errors added by the cte in every part of p, i, d of the controller. The `TotalError` (Lines 110-114) calculates the steering from the updated errors.

## Setting Hyper-parameters

The hyper parameters are set in different ways to understand the effect of each of them the following file show effects of each of them.

### P parameter

- Setting the P to a higher value

![higher_p](images/p_100.gif)

The P value reacts proportionally to the error. Higher P leads to bigger changes or more reaction in every correction.

- Setting the P to a lower value

![lower_p](images/p_20.gif)

The P value reacts proportionally to the error. Lower P leads to smaller changes or less reaction in every correction.

### I parameter

I parameter corrects the residual errors or historic cumulative value of error. This should be always set to small values.

- Setting the I parameter to a higher value

![higher_i](images/i_0.01.gif)

This changes the amplitude for the oscillation. Higher I leads to smaller amplitude for the oscillation to correct the error.

- Setting the I parameter to a lower value

![lower_i](images/i_0.001.gif)

This changes the amplitude for the oscillation. Lower I leads to higher amplitude for the oscillation to correct the error.

### D parameters

D parameter is connected with the rate of change.

- Setting the D parameter to a higher value

![higher_d](images/d_100.gif)

This makes the controller respond to the changes faster, the higher the d, the faster the controller tries to correct the error.

- Setting the D parameter to a lower value

![lower_d](images/d_20.gif)

This makes the controller respond to the changes slower, the lower the d, the slower the controller tries to correct the error.

## Using Twiddle to find the hyper-parameters

In the `PID.cpp` the twiddle function was also implemented (Lines 32-80). It can be started with a boolean flag in the `main.cpp`. In contrast to the Udacity implementation of Twiddle, I used a simple state machine, so my main file stays simple. The first parameter of the Twiddle are important here. In Udacity implementation in python the system had no boundary (road boundaries) so at some points the system would converge to the right parameters. In this assignment the road boundaries made the twiddle function struggle. So the first parameters set to the twiddle function should be set in a manner that the car does not collide or stays on the boundaries. With experimentation, this parameters are found. Then with the help of twiddle better parameters are calculated. What is delivered by twiddle is not the best parameter and still can be optimized.