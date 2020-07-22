# Kinematic-Solver
This project is a Kinematic solver of planar robotic manipulators. Currently this has two models: A 2 Link Manipulator and a 3 Link Manipulator. I intend to commit few more models to this repository in the future.

# Overview - 2 Link Manipulator
The inverse kinematics of this manipulator are analytical. To address the issue of multiple solutions for a given pose the joint configuration with minimum displacement from the previous position is chosen.

# Overview - 3 Link Manipulator
The inverse kinematics of this manipulator are based on Jacobians. An iterative solver is employed which checks for the reference position on the basis of increments and minimization of Eucledian Distance between the reference and actual position of end effector. Few parameters are included in the solver which impact the learning curve and ability of the solver to converge to feasible solution.

# Reference Tracking
The manipulators are shown for two applications: 
1. Reference Trajectory Tracking:
    1. Circular Trajectory
    1. Eight Shaped Trajectory
1. Point to Point Motion.

# Run
To run the code select the code file with the intended application, i.e, Trajectory Tracking or Point to Point Motion. Adjust the tunable parameters (if required). The details about link lengths and trajectory are taken as user input.

# Example

## 2 Link Manipulator
### 1. Point to Point Motion

