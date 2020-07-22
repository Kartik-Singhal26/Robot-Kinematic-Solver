# -*- coding: utf-8 -*-
"""
Created on Tue Jul 21 20:13:35 2020

@author: kartik
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

# Workspace
Lm = L1 + L2
n = 100

# Setup Animation Environment
fig = plt.figure()
anim = fig.add_subplot(111, aspect = 'equal', autoscale_on = False, xlim = (-20, 20), ylim = (-20, 20))
anim.grid(color = 'grey', linestyle = '-', linewidth = 0.5)  
plt.title('Kinematic Solver | Point To Point Tracking') 
link1, = anim.plot([], [], 'o-', lw = 4, mew = 5)
link2, = anim.plot([], [], 'o-', lw = 4, mew = 5)
link1trace, = anim.plot([], [], '-', lw = 0.5, mew = 1)
link2trace, = anim.plot([], [], '-', lw = 0.5, mew = 1)
time_template = 'Time = %.1fs'
time_text = anim.text(0.02, 0.95, '', transform = anim.transAxes)
Joint1_template = 'Joint 1 Pose = %.1f degree'
Joint2_template = 'Joint 2 Pose = %.1f degree'
Joint1_text = anim.text(0.02, 0.90, '', transform = anim.transAxes)
Joint2_text = anim.text(0.02, 0.85, '', transform = anim.transAxes)

dt = 1./50

# Plot Workspace
Coord = PointsInCircle(Lm,n)
xc = [ p[0] for p in Coord ]
yc = [ p[1] for p in Coord ]
i = len(xc)

plt.plot(xc,yc)

# Take input coordinates for Tracking
i = 15
c = plt.ginput(i)  
X = []
Y = []

for k in range(i):
    x = c[k][0]
    y = c[k][1]
    X.append(x)
    Y.append(y)  

# Find Kinematic Solution
Conf = Configuration(L1, L2, X, Y, i)
x1 = Conf[0]
y1 = Conf[1]
x2 = Conf[2]
y2 = Conf[3]
J1 = Conf[4]
J2 = Conf[5]

# Update the Plot

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

ani = animation.FuncAnimation(fig, animate, np.arange(1, i), interval = interval, blit = True, init_func = init)

ani.save('P2P_Tracking.mp4', fps = 10)
plt.show()
