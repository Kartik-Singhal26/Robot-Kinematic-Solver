# -*- coding: utf-8 -*-
"""
Created on Tue Jul 21 20:13:34 2020

@author: kartik

This file plots and animate the 2 link manipulator for trajectory tracking application.

"""

import numpy as np
from time import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from two_link_planar import Configuration
from Reference_Trajectory import PointsInCircle

# #Link Length
L1 = int(input('Enter length of Link 1: '))
L2 = int(input('Enter Length of Link 2: '))
r = int(input('Enter Radius of Trajectory: '))
n = 100

Coord = PointsInCircle(r,n)
X = [ c[0] for c in Coord ]
Y = [ c[1] for c in Coord ]
i = len(X)

#Plot the mechanism
temp = Configuration(L1, L2, X, Y, i)
x1 = temp[0]
y1 = temp[1]
x2 = temp[2]
y2 = temp[3]
J1 = temp[4]
J2 = temp[5]

fig1 = plt.figure()
plot1 = fig1.add_subplot(111, aspect = 'equal', autoscale_on = False, xlim = (-20, 20), ylim = (-20, 20))
locs, labels = plt.xticks()
plot1.plot(x1,y1)
plot1.plot(x2,y2)
Link1, = plot1.plot([0,x1[-1]],[0,y1[-1]], lw=4, mew=5, label='Link 1', color = 'Orange')
Link2, = plot1.plot([x1[-1],x2[-1]],[y1[-1],y2[-1]], lw=4, mew=5, label='Link 2', color = 'Blue')
plot1.legend(handles = [Link1, Link2])
plot1.grid(color = 'grey', linestyle = '-', linewidth = 0.5)
#plt.autoscale(enable = True, axis = 'both', tight = 'None')   

#______________________________________________________________________________

#Animate the plot

# Setup Animation Environment
fig2 = plt.figure()
anim = fig2.add_subplot(111, aspect = 'equal', autoscale_on = False, xlim = (-20, 20), ylim = (-20, 20))
anim.grid(color = 'grey', linestyle = '-', linewidth = 0.5)  
plt.title('Kinematic Solver | Trajectory Tracking') 
link1, = anim.plot([], [], 'o-', lw = 4, mew = 5, label = "Link 1", color = 'Orange')
link2, = anim.plot([], [], 'o-', lw = 4, mew = 5, label = "Link 2", color = 'Blue')
link1trace, = anim.plot([], [], '-', lw = 0.5, mew = 1, color = 'Orange')
link2trace, = anim.plot([], [], '-', lw = 0.5, mew = 1, color = 'Blue')
time_template = 'Time = %.1fs'
time_text = anim.text(0.02, 0.95, '', transform = anim.transAxes)
Joint1_template = 'Joint 1 Pose = %.1f degree'
Joint2_template = 'Joint 2 Pose = %.1f degree'
Joint1_text = anim.text(0.02, 0.90, '', transform = anim.transAxes)
Joint2_text = anim.text(0.02, 0.85, '', transform = anim.transAxes)

dt = 1./100 

#plot workspace
plt.plot(X,Y)

def init():
    link1.set_data([], [])
    link2.set_data([], [])
    
    time_text.set_text('')
    Joint1_text.set_text('')
    Joint2_text.set_text('')
    return link1,link2,Joint1_text,Joint2_text, time_text

def animate(a):
    link1_x = x1[a] 
    link1_y = y1[a]
    link2_x = x2[a] 
    link2_y = y2[a]
    
    link1.set_data([0,link1_x],[0,link1_y])
    link2.set_data([link1_x,link2_x],[link1_y,link2_y])
    
    link1trace.set_data(x1,y1)
    link2trace.set_data(x2,y2)
    
    time_text.set_text(time_template % (a*dt))
    Joint1_text.set_text(Joint1_template % (J1[a]))
    Joint2_text.set_text(Joint2_template % (J2[a]))
 
    return link1, link2, Joint1_text, Joint2_text, time_text

t0 = time()
animate(0)
t1 = time()
interval = 10000 * dt - (t1 - t0)

ani = animation.FuncAnimation(fig2, animate, np.arange(1, 100), interval = interval, blit = True, init_func = init)

ani.save('Trajectory_Tracking.mp4', fps=10)
plt.show()
