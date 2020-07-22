"""
Created on Tue July 21 21:26:27 2020

@author: kartik

This file contains forward and inverse kinematics code for a 
three link planar Manipulator

Following Assumptions are made for the model:

1. Theta 1 is angle between arm one and horizontal 
2. Theta 2 is angle between arm one and arm two    
3. Theta 3 is angle between arm two and arm three
4. The Manipulator is situated at origin

"""
import math as m
import numpy as np
from random import random

# Define Forward Kinematics

def ForwardKin(L1, L2, L3, T1, T2, T3):
    
    '''   
    This function gives end effector position
    It takes 6 arguments:
    Where,
    Li = length of Link i 
    Ti = Angular Position of i Link in radians
    '''   
   
    # Link coordinates of Arm 1
    x1 = L1*m.cos(T1) 
    y1 = L1*m.sin(T1)   

    # Link Coordinates of Arm 2
    x2 = x1 + L2*m.cos(T1 + T2) 
    y2 = y1 + L2*m.sin(T1 + T2)
    
    # Link Coordinates of Arm 3
    x3 = x2 + L3*m.cos(T1 + T2 + T3) 
    y3 = y2 + L3*m.sin(T1 + T2 + T3)

    Sol = {'Link1' : (x1,y1), 'Link2' : (x2,y2), 'Link3' : (x3,y3)}        
    return(Sol)

def InvKinematics(L1, L2, L3, X, Y):
    
    '''
    This function returns set of possible solutions for the joint angles
    The inverse kinematics are calculated on the basis of the Jacobian Matrix
    
    Here, 
    L1, L2, L3 are link lengths.
    X and Y are desired Coordinates
    '''
    # Initialize Theta Variables (In radians)
    
    Th1 = 0.1*m.pi/180
    Th2 = 0.1*m.pi/180
    Th3 = 0.1*m.pi/180
    
    #Theta_vect = [Th1, Th2, Th3]
    Goal = np.array([[X], [Y]])
    
    # Calculate Jacobian Matrix and related vectors
    
    def CalcJacobian(L1, L2, L3, Th1, Th2, Th3): 
        S1 = m.sin(Th1)
        S12 = m.sin(Th1 + Th2)
        S123 = m.sin(Th1 + Th2 + Th3)
        C1 = m.cos(Th1)
        C12 = m.cos(Th1 + Th2)
        C123 = m.cos(Th1 + Th2 + Th3)
    
        Jacobian = np.matrix([[-(L1*S1 + L2*S12 + L3*S123), -(L2*S12 + L3*S123), -L3*S123],
                              [(L1*C1 + L2*C12 + L3*C123), (L2*C12 + L3*C123), L3*C123]])
    
        return Jacobian
    
    errorThresh = 0.01 # Error Threshold value
    K = 0.003 # learning rate
    #print('I am at Inverse Kinematics')
           
    CurrentPos = ForwardKin(L1, L2, L3, Th1, Th2, Th3)
    Xc = CurrentPos['Link3'][0]
    Yc = CurrentPos['Link3'][1]
    D = m.sqrt((Xc - X)**2 + (Yc - Y)**2)
    
    # Iterative Correction
    counts = 0
    max_iter = 500 # Adjust this if solution is not available 
    Errorlog = []
    while D > errorThresh:
        
        Jacobian = CalcJacobian(L1, L2, L3, Th1, Th2, Th3)
        
        # Simple Jacobian Inverse Method -> Requires conversion of Jacobian to Square Matrix 
        #Jacobian_inv = np.linalg.inv(Jacobian)  
        
        # Pseudo Inverse Method -> Works for Non square Matrix
        
        #Jacobian_pseudoinv = np.dot(np.linalg.inv(np.dot(Jacobian.T, Jacobian)), Jacobian.T)
        
        # Transpose Method -> Simple Transpose of Jacobian Matrix instead of inverse
        # Note -> In my tests, Jacobian Transpose has given best results with least run time
        
        Jacobian_transpose = Jacobian.T
        
        adjust = np.array([[(X - Xc)], [(Y - Yc)]]) # Error in Positions of End Effector
    
        # Angular Displacement
    
        delta_theta = Jacobian_transpose * K * adjust # Angular Displacement Vector
        Th1 += delta_theta[0]
        Th2 += delta_theta[1]
        Th3 += delta_theta[2]
    
        # Update Current Position and Theta Vector
    
        CurrentPos = ForwardKin(L1, L2, L3, Th1, Th2, Th3)
        Xc = CurrentPos['Link3'][0]
        Yc = CurrentPos['Link3'][1]
        
        Theta_vect = [Th1, Th2, Th3]
        
        # Distance between the current and desired position of end effector
    
        D = m.sqrt((Xc - X)**2 + (Yc - Y)**2)
        Errorlog.append(D)
        
        print('Iteration Count: ', counts)
        print('Still Correcting. D = ', D)
        
        # Update Parameters
        counts += 1
        
        #Avoid Infinite Loop
        if counts > max_iter:
            print('Solution is not optimum')
            break
        
    else:
        print('Joint Configuration Achieved')
        
    return Theta_vect, Errorlog
       

     
        
        
                         
                         
    
    
