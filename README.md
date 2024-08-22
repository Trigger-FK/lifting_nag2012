# lifting_nag2012
This repository include the simulation code of optimal regulator via lifting technique.

_reference_
永原, “離散時間制御”, システム／制御／情報, 56 (6), 2012

## Description
### cart_ipd_control.m
MATLAB code for example 1.
In this file, you can compare the step response of an I-PD control system between the results of continuous-time control and control with bilinear transform.

### ipd_controller.slx
Simulink file for `cart_ipd_control.m`

### ipd_lifting_regulator.m
MATLAB code for example 2.
In this file, you can Compare results for system response with an optimal regulator between designs with lifting and designs that discretize a continuous-time controller.

### ipd_controller.slx
Simulink file for `ipd_lifting_regulator.m`

