# Planning Throwing Motion for Mobile Manipulators
## Project Description
The aim of this project is to plan a throwing motion for a planar mobile manipulator (MM), in an attempt to increase its workspace or reduce the time needed for a pick and place task. The motion planning problem is formulated as an Optimal Control Problem (OCP) and solved using numerical optimization via the Optimization Toolbox of MATLAB. Robot balance is guaranteed along the whole planned trajectory using an appropriate nonlinear constraint based on the MM full dynamics, ensuring non-negative moments around the edges of the support polygon.

The OCP to solve is the following
```math
\begin{align*}
&\min_{\mathbf{w}} \sum^{N-1}_{i=0}(\dot{\mathbf{q}}_{r,i}^T\mathbf{W}\dot{\mathbf{q}}_{r,i}+x_{b,i}^2\rho+\mathbf{u}_i^T\mathbf{R}\mathbf{u}_i)+\dot{\mathbf{q}}_{r,N}^T\mathbf{W}\dot{\mathbf{q}}_{r,N}\\
&\text{subject to:}\nonumber\\
&\mathbf{x}_{i+1}-\mathbf{\phi}^l_{d\text{-}t}(\mathbf{x}_i,\mathbf{u}_i)=\ \mathbf{0} \quad i=0 \dots k-1\\
&\mathbf{x}_{i+1}-\mathbf{\phi}_{d\text{-}t}(\mathbf{x}_i,\mathbf{u}_i)=\ \mathbf{0} \quad i=k \dots N-1\\ 
&g(\mathbf{x}_k)=0 \\
&\mathbf{x}_0-\mathbf{x}_{init}=\ \mathbf{0} \\
&\dot{\mathbf{q}}_{r,N}=\ \mathbf{0} \\
&\mathbf{x}_{min} \le \mathbf{x}_i \le \mathbf{x}_{max} \quad i=0 \dots N \\
&\mathbf{u}_{min} \le \mathbf{u}_i \le \mathbf{u}_{max} \quad i=0 \dots N-1 \\
&\mathbf{b}^l(\mathbf{x}_i,\mathbf{u}_i) \ge \mathbf{0} \quad i=0 \dots k-1 \\
&\mathbf{b}(\mathbf{x}_i,\mathbf{u}_i) \ge \mathbf{0} \quad i=k \dots N-1 \ .
\end{align*}
```
The cost function is composed of four terms: we have the running and terminal cost associated to the joint velocity $\dot{\mathbf{q}}$ that are damping terms, the cost associated to the control effort with its weighting matrix $\mathbf{R}$, and the term weighting the position of the robot base $x_b$ with weight $\rho \in \mathbb{R}$ that penalizes the distance of the robot base from the origin. This term leads to a solution where the robot throws the object limiting base movements towards the goal. In the constraints, $\phi_{d\text{-}t}(\cdot,\cdot)$ is the discrete-time dynamics obtained with the 4th order Runge-Kutta integration method, $\mathbf{b}(\cdot,\cdot)$ is the balance constraint, and the superscript $l$ indicates whether we are considering the robot dynamics with the load attached to the end-effector or not. Finally, $k=T_{t}/\delta$ is the index that identifies the throwing instant, $g(\cdot)=0$ is the ballistic constraint and $\mathbf{x}_{init}$ is an arbitrary resting initial state;

The following video, available also in the ```media``` folder, shows the obtained solution and the influence of the base penalization parameter $\rho$

https://github.com/lucanunz/Mobile_Manipulator_Throwing_Motion/assets/72447693/8ab48420-8e54-4cb1-b80d-a0626376cc22


More details are available in the pdf, and an extended version of this video, with additional simulations, is available [here](https://drive.google.com/drive/folders/1KANx67GlRE-5K6ukSk9xtMDbt_H63qQR).

## Main files
The ```source``` folder contains
  -	```dyn_model_complete.m``` derives the dynamic model of the robot. It generates 3 functions:
    - ```get_balance_terms.m``` used to formulate the balance constraint
    - ```get_dyn_terms.m``` used for the equations of motion in ct_dynamics.m
    - ```get_gen_forces.m``` that computes the reaction forces used to find the ZMP
  - ```optimization.m``` solves the Non Linear Program
  - ```dt_dynamics.m``` discretizes the dynamics with 4th-order Runge-Kutta

A skect of the Mobile Manipulator, along with the kinematic and dynamic quantities that are specified in the scripts, can be found in ```media/MM_measures.PNG```.
 
