# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
<iframe width="560" height="315"
src="https://www.youtube.com/embed/T8NowzsJlyQ" 
frameborder="0" 
allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" 
allowfullscreen></iframe>

[image1]: ./result/pid.png "pid diagram"
## Overview
This project is to understand the effect of the P, I, D component of the PID algorithm, and implement it to drive a virtual car in the simulator. The car is expected to stay in the middle of the lane, and adjust properly when making turns.

## Content
* README.md: the report
* src: source code
* ProjectDescription: the original readme file from Udacity

## Compilation
1. Make a build directory: `mkdir build && cd build`
2. Compile: `cmake .. && make`
3. Run it: `./pid`. 

## Implementation and Reflection
The following figure is generated from the sample code of this course. It explains itself very well. 
For PID controller, `P` is proportional to the cross track error (i.e. CTE), and `P` may have the issue of oscillation or ring back. 
`D` is the difference between the current CTE and the preivous CTE. `D` can help to reduce the oscillation and help it approach the center.
`I` is the integral or sum of the CTE. `I` can help to reduce the system bias.

![alt text][image1]


The steering angle is set by the parameters as below (line 146 in `src/main.cpp` and line 86 - 91 in `PID.cpp`):

```steering = -tau_p * CTE - tau_d * diff_CTE - tau_i * int_CTE```

The PID variables are initialized in line 38 - 59 in `src/main.cpp`. After some trial, I manually chose `0.2, 0.004, 3.0` for parameters of P, I, and D (line 43 in `src/main.cpp`).

Then Twiddle algorithm is used to optimize the P, I, D parameters (line 90 - 143 `src/main.cpp`). A boolean value `twiddle` is to turn on or off the twiddle optimization (line 45).

After Twiddle algorithm, the parameters are changed to `0.21, 0.004, 3.728`.

## Simulation
The car can drive successfully on the track, and stay around the center of the track.

