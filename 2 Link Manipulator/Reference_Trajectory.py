# -*- coding: utf-8 -*-
"""
Created on Wed May 13 14:29:51 2020

@author: kartik

This file generates reference trajectory

function PointsinCircle generates coordinates for circumference of a given circle
r = radius of circle
n = number of generated points
"""
import math as m
pi = m.pi

# Circular Trajectory

def PointsInCircle(r,n):
    return [((m.cos(2*pi/n*x)*r), (m.sin(2*pi/n*x)*r)) for x in range(0,n+1)]

# Eight Shaped Trajectory

def PointsInEight(r,n):
    return [((m.cos(2*pi/n*x)*r), ((m.sin(2*2*pi/n*x))*r/2)) for x in range(0,n+1)]


