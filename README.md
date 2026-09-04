# Self-Driving Car Controls

MATLAB control-systems project for designing a lateral position controller for a self-driving car.

## Objective

Design a controller that satisfies:

Overshoot: < 10%

Maximum steering angle: < 4°

## Model

```text
   R(s) +   E(s)               U(s)       X(s) 
   -----> ○ ----> Gc ----> Ga ----> Gp --┬-->
        - ↑                              |
          └------------------------------┘
```

The system uses a proportional controller:

𝐺𝑐(𝑠) = 𝐾

with:

𝐺𝑎(𝑠) = 10𝑠 + 10

𝐺𝑝(𝑠) = 0.1𝑠 (𝑠 + 1)

The MATLAB script analyzes the system using root locus, step response, gain sweeps, and Bode plots.

## Results

The final controller gain is:

K = 4.106

```text
Overshoot	= 2.93%
Rise Time =	3.57 s
Max Steering Angle	≈ 4°
Bandwidth	= 0.602 rad/s
```

The 4° steering constraint is the limiting requirement.

Requirements
MATLAB
Control System Toolbox

Run self_driving_car_controls.m to generate the analysis plots.
