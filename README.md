# Kinematic-Solver
This project is a Kinematic solver of a 2 link robotic manipulator in a 2D Plane. To test the solver, run the file Plot2link.py. The link lengths are taken as user inputs. By default the project shows an interactive plot which takes 10 target coordinates as input for point to point motion of the manipulator.

# two_link_planar.py 
This file contains functions for calculating forward and inverse kinematics of the manipulator, it returns two posssible solution sets for each target.

# PossibleConfiguration.py 
This file returns a solution from the possible set by comparing the joint distance between two successive targets. The pose with minimum joint distance is returned.

# Reference_Trajectory.py 
It contains functions to plot a circular trajectory or a eight shaped trajectory

# Plot2link.py 
It contains two sections:
1. Plot the mechanism
It simply shows the manipulator's end position in a non interactive plot
2. Animate
It takes n number of target coordinates from the user and then shows an animated plot of the manipulator iterating through the targets. [Point to Point Motion] 
