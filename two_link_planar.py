# -*- coding: utf-8 -*-
"""
Created on Tue May 12 07:37:29 2020

@author: kartik

This file contains forward and inverse kinematics code for a 
two link planar Manipulator

Following Assumptions are made for the model:

1. Theta1 is angle between arm one and horizontal 
2. Theta2 is angle between arm one and arm two    
3. The Manipulator is situated at origin
"""
import math as m

#Define Forward Kinematics

def forwardkin(L1,L2,T1,T2):
    
    '''   
    This function gives end effector position
    It takes 4 arguments:
    L1 = length of Link 1 
    L2 = length of Link 2 
    T1 = Initial Position of Link 1 in degrees
    T2 = Initial Position of Link 2 in degrees
    '''   
    #Convert degrees into radian
    T1 = T1*(m.pi)/180
    T2 = T2*(m.pi)/180
    
    # Link coordinates of Arm 1
    x1 = L1*m.cos(T1) 
    y1 = L1*m.sin(T1)   

    # Link Coordinates of Arm 2
    x2 = x1 + L2*m.cos(T1 + T2) 
    y2 = y1 + L2*m.sin(T1 + T2) 

    Sol = {'Link1' : (x1,y1), 'Link2' : (x2,y2)}        
    return(Sol)
             
#Define Inverse Kinematics
    
def invkin(L1,L2,X,Y):
    '''   
    This function gives Joint Angles and link endpoints for a 2 link planar manipulator
    It takes 4 arguments:
    L1 = length of Link 1 
    L2 = length of Link 2 
    X = End Effector Position in X axis
    Y = End effector Position in Y axis
    '''       
    #Using Analytical Approach
    
    c = ((X**2 + Y**2 - L1**2 - L2**2)/(2*L1*L2))
    
    if c**2 <= 1:
        print('Target is reachable')
        
        s_p = m.sqrt((1 - (c**2)))
        s_n = -(s_p)
        #s = [s_p, s_n]
        
        K1 = L1 + c*L2
        K2_a = s_p*L2
        K2_b = s_n*L2
        #K2 = [K2_a, K2_b]
        
        #Possible Joint 2 angles (in radians) 
        T2_a = m.atan2(s_p,c)
        T2_b = m.atan2(s_n,c)
                
        #Possible Joint 1 angles (in radians)
        T1_a = m.atan2(Y,X) - m.atan2(K2_a,K1)
        T1_b = m.atan2(Y,X) - m.atan2(K2_b,K1)
        
        #Angles converted to degrees
        T1_ad = T1_a*180/(m.pi) #Converted to degrees
        T2_ad = T2_a*180/(m.pi) #Converted to degrees
        T1_bd = T1_b*180/(m.pi) #Converted to degrees
        T2_bd = T2_b*180/(m.pi) #Converted to degrees
        
        Sol1 = forwardkin(L1,L2,T1_ad,T2_ad)
        Sol1['Pos'] = (T1_ad,T2_ad)
        Sol2 = forwardkin(L1,L2,T1_bd,T2_bd)
        Sol2['Pos'] = (T1_bd,T2_bd)
        return(Sol1,Sol2)
    else:
        print('Target is not reachable, adjust link length.')

# Find solution set

def Configuration(L1, L2, X, Y, i):
    
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
    
    Solutions = []
    for j in range(i):
        Sol = invkin(L1, L2, X[j], Y[j])
        Solutions.append(Sol)
     
    # EndEffector Contains Optimum Solution Sets    
        
    EndEffector_Pos = []
    
    def Calc_Distance():
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
    
    Calc_Distance()
    # print('You did it bitch')
   
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
        

