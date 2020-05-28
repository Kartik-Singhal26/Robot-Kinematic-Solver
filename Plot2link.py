# -*- coding: utf-8 -*-
"""
Created on Wed May 13 14:56:32 2020

@author: kartik

This function animates the 2 link manipulator
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from PossibleConfiguration import *
from time import time
# #Link Length
L1 = int(input('Enter length of Link 1: '))
L2 = int(input('Enter Length of Link 2: '))
r = (L1+L2)
n = 100
# '''
# X = int(input('End Effector Position in X axis: '))
# Y = int(input('End Effector Position in Y axis: '))
# '''
#r = int(input('Radius of Reference Circle or Eight Shaped Trajectory: '))
'''
#Plot the mechanism
temp = Configuration(L1,L2,r)
x1 = temp[0]
y1 = temp[1]
x2 = temp[2]
y2 = temp[3]
J1 = temp[4]
J2 = temp[5]
plt.figure()
locs, labels = plt.xticks()
plt.plot(x1,y1)
plt.plot(x2,y2)
Link1, = plt.plot([0,x1[-1]],[0,y1[-1]], label='Link 1')
Link2, = plt.plot([x1[-1],x2[-1]],[y1[-1],y2[-1]], label='Link 2')
plt.legend(handles = [Link1, Link2])
plt.grid(color='grey', linestyle='-', linewidth=0.5)
plt.autoscale(enable = True, axis = 'both', tight = 'None')   
'''
#______________________________________________________________________________


Coord = PointsInCircle(r,n)
xc = [ c[0] for c in Coord ]
yc = [ c[1] for c in Coord ]
i = len(xc)

#Animate the plot

# Setup Animation Environment
fig = plt.figure()
anim = fig.add_subplot(111, aspect='equal', autoscale_on = False, xlim=(-15, 15), ylim=(-15, 15))
anim.grid(color='grey', linestyle='-', linewidth=0.5)  
plt.title('Kinematic Solver for 2 Link Manipulator') 
link1, = anim.plot([], [], 'o-', lw=4, mew=5)
link2, = anim.plot([], [], 'o-', lw=4, mew=5)
link1trace, = anim.plot([], [], '-', lw=0.5, mew=1)
link2trace, = anim.plot([], [], '-', lw=0.5, mew=1)
time_template = 'Time = %.1fs'
time_text = anim.text(0.02, 0.95, '', transform = anim.transAxes)
Joint1_template = 'Joint 1 Pose = %.1f degree'
Joint2_template = 'Joint 2 Pose = %.1f degree'
Joint1_text = anim.text(0.02, 0.90, '', transform = anim.transAxes)
Joint2_text = anim.text(0.02, 0.85, '', transform = anim.transAxes)

dt = 1./100 #50 fps
# Initialize background of each frame in plot

i = 10
c = plt.ginput(i) # Take input coordinates 
X = []
Y = []

for k in range(i):
    x = c[k][0]
    y = c[k][1]
    X.append(x)
    Y.append(y)    

Conf = Configuration(L1, L2, X, Y)
x1 = Conf[0]
y1 = Conf[1]
x2 = Conf[2]
y2 = Conf[3]
J1 = Conf[4]
J2 = Conf[5]

#plot workspace
plt.plot(xc,yc)
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

ani = animation.FuncAnimation(fig, animate, np.arange(1, 10), interval = interval, blit = True, init_func = init)

ani.save('Kinematic_Solver.mp4', fps=5)
plt.show()

#______________________________
