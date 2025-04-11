# Matlab Trajectory Planner
This repository contains Matlab scripts and functions for the computation of motion trajectories using 
polynomials methods as presented in [1].

## Point-To-Point Motion
The point-to-point trajectory planning is implemented using a cubic and a polynomial motion law. The implementation is made by using Matlab functions
`compute_cubic.m` and `compute_quintic.m`.
The inputs of this functions are $t_i$ and $t_f$ that are the initial and final time respectively, $q_i$ the starting configuration and $q_f$ that is the final configuration. The `do_plot` boolean flag is used to choose if plot of the trajecotry is needed or not. $q_i$ and $q_f$ quantities are $\in \mathbb{R}^2$ for the cubic trajectory, where only initial and final position and velocity can be imposed, while are $\in \mathbb{R}^3$ in the quintic trajectory.

## Operational Space Trajectory
Two Matlab functions implement the operational space trajectories, `task_space_traj.m` and `task_space_circular.m` for rectilinear and circular paths respectively. These functions employ the quintic functions above-mentioned. The inputs for the rectilinear path function are $t_i$ that is the trajectory starting time (scalar), $t_f$ trajectory final time (scalar), $T_i$ is the transformation matrix of the initial pose and $T_f$ the transformation matrix of the final pose. The inputs of the circular path function differ from the other only in the number of the transformation matrices, the input transformation matrices are required to define the circumference of the trajectory. The outputs are $T_d$, vector of transformation matrices through the whole trajectory and $\dot{T}_d$. As optional inputs there are the linear acceleration, angular velocity and angular acceleration.
Two scripts are provided as example for task space trajectory generation, `square_traj.m` and `circular_traj.m` that generate a square and circular trajectory.


## Bibliography

[1] B. Siciliano, L. Sciavicco, L. Villani, and G. Oriolo, *Robotics: Modelling, Planning and Control*, 2008, Springer Publishing Company, Incorporated, 1st edition.
