# Planning Throwing Motion for Mobile Manipulators
This repository contains a MATLAB implementation of planning a throwing motion for a planar mobile manipulator via offline optimal control.

The ```source``` folder contains
  -	```dyn_model_complete.m``` derives the dynamic model of the robot. It generates 3 functions:
    - ```get_balance_terms.m``` used to formulate the balance constraint
    - ```get_dyn_terms.m``` used for the equations of motion in ct_dynamics.m
    - ```get_gen_forces.m``` that computes the reaction forces used to find the ZMP
  - ```optimization.m``` solves the Non Linear Program
  - ```dt_dynamics.m``` discretizes the dynamics with 4th-order Runge-Kutta

A skect of the Mobile Manipulator, along with the kinematic and dynamic quantities that are specified in the scripts, can be found in ```media/MM_measures.PNG```.
