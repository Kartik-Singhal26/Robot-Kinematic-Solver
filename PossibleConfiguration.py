# -*- coding: utf-8 -*-
"""
Created on Wed May 13 14:58:46 2020

@author: kartik

This program filter the available solutions to return a single set
"""

#import numpy as np
import math as m
from Reference_Trajectory import *
from two_link_planar import *

#Enter Link Length and Desired Position Coordinates

# #Link Length
# L1 = int(input('Enter length of Link 1: '))
# L2 = int(input('Enter Length of Link 2: '))
# '''
# X = int(input('End Effector Position in X axis: '))
# Y = int(input('End Effector Position in Y axis: '))
# '''
# r = int(input('Radius of Reference Circle or Eight Shaped Trajectory: '))

# Enter Initial Position of the End effector

 #Generate Reference Trajectory [r = user input, n = 100]
# n = 100 #Number of Coordinate Points
# Coord = PointsInCircle(r,n)
# X = [ c[0] for c in Coord ]
# Y = [ c[1] for c in Coord ]
# i = len(X)
    
    
def Configuration(L1, L2, X, Y):
    
   
    '''
    The function invkin(L1, L2, X, Y,i) gives Joint Angles and link endpoint 
    Positions for a 2 link planar manipulator
    It takes 4 arguments:
    L1 = length of Link 1 
    L2 = length of Link 2 
    X = End Effector Position in X axis
    Y = End effector Position in Y axis
    i = Number of data points for trajectory
    '''
    i = 10
    Solutions = []
    for j in range(i):
        Sol = invkin(L1, L2, X[j], Y[j])
        Solutions.append(Sol)
     
    # EndEffector Contains Optimum Solution Sets    
        
    EndEffector_Pos = []
    
    def calc_distance():
        '''
        This function calculates the distance to be travelled by both 
        joints to move to next position. Then the solution set with the minimum
        distance travelled is selected.
        
        '''
        
        # Initial Pose/Resting Position of Manipulator
        x0_1 = L1
        y0_1 = 0
        x0_2 = L1 + L2
        y0_2 = 0
    
        # Weights for Joint Displacement
        w1 = 0.5 #For Joint 1
        w2 = 0.5 #For Joint 2
        
        for k in range(i):
            
            #Extract both possible pose from the list Solutions
            Pos1 = Solutions[k][0]  #Pos1
            Pos2 = Solutions[k][1]  #Pos2
            
            #Extract X & Y Coordinates for Link1 & Link2 Positions
            
            #Pos 1
            x_link1_a = Pos1["Link1"][0] #Link1 | Pose 1 : X Coordinate
            y_link1_a = Pos1["Link1"][1] #Link1 | Pose 1 : Y Coordinate
            x_link2_a = Pos1["Link2"][0] #Link2 | Pose 1 : X Coordinate
            y_link2_a = Pos1["Link2"][1] #Link2 | Pose 1 : Y Coordinate
            
            #Pos 2 
            x_link1_b = Pos2["Link1"][0] #Link1 | Pose 2: X Coordinate
            y_link1_b = Pos2["Link1"][1] #Link1 | Pose 2: Y Coordinate
            x_link2_b = Pos2["Link2"][0] #Link2 | Pose 2: X Coordinate
            y_link2_b = Pos2["Link2"][1] #Link2 | Pose 2: Y Coordinate
            
            #Calculate Joint Displacement in Pos 1
            r_link1_a = m.sqrt((x_link1_a - x0_1)**2 + (y_link1_a - y0_1)**2) #Joint Displacement for Link 1 in Pose 1
            r_link2_a = m.sqrt((x_link2_a - x0_2)**2 + (y_link2_a - y0_2)**2) #Joint Displacement for link 2 in Pose 1
         
            #Cummulative Joint Displacement in Pose 1
            r_a = w1*r_link1_a + w2*r_link2_a     
            
            #Calculate Joint Displacement in Pos 2
            r_link1_b = m.sqrt((x_link1_b - x0_1)**2 + (y_link1_b - y0_1)**2) #Joint Displacement for Link 1 in Pose 2
            r_link2_b = m.sqrt((x_link2_b - x0_2)**2 + (y_link2_b - y0_2)**2) #Joint Displacement for Link 2 in Pose 2
            
            #Cummulative Joint Displacement in Pose 2  
            r_b = w1*r_link1_b + w2*r_link2_b  
            
            #Filter the Pose with minimum of the both displacement
            if r_a > r_b:
                EndEffector_Pos.append(Pos2) #Update the End Effector Position Coordinates
                x0_1 = x_link1_b             #Update Initial Pose for next solution set
                y0_1 = y_link1_b
                x0_2 = x_link2_b
                y0_2 = y_link2_b
            else:
                EndEffector_Pos.append(Pos1) #Update the End Effector Position Coordinates
                x0_1 = x_link1_a
                y0_1 = y_link1_a
                x0_2 = x_link2_a
                y0_2 = y_link2_a
                
    #___________________________________________________________
    
    calc_distance()
    print('You did it bitch')
    #Get Link Coordinates for Plotting
    #Coordinates of Link1
    x1 = [] 
    y1 = []
    J1 = []
    #Coordinates of Link2
    x2 = []
    y2 = []
    J2 = []
    for k in range(i):
        x1.append(EndEffector_Pos[k]['Link1'][0])
        y1.append(EndEffector_Pos[k]['Link1'][1])
        x2.append(EndEffector_Pos[k]['Link2'][0])
        y2.append(EndEffector_Pos[k]['Link2'][1])
        J1.append(EndEffector_Pos[k]['Pos'][0])
        J2.append(EndEffector_Pos[k]['Pos'][1])
    
    return x1, y1, x2, y2, J1, J2