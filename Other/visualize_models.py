# -*- coding: utf-8 -*-
"""
Created on Thu Dec 13 10:35:34 2018

@author: bimta
"""

from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
from operator import itemgetter
import matplotlib.pyplot as plt
import numpy as np

cyl_points = []
for cyl in cylinders:
    points = cyl.to_array()
    cyl_points.append(points)


def findCoordinate(cylinder, model):
    points = cylinder.to_array()
    lowest_point = points[np.where(points[:,2] == points[:,2].min())]
    z = lowest_point[:,2]
    
    xp = model[0]
    yp = model[1]
    zp = model[2]
    
    xd = model[3]
    yd = model[4]
    zd = model[5]
    
    bla = (z - zp)/zd  
    x = (bla * xd) + xp
    y = (bla * yd) + yp
    return x, y, z

x, y, z = findCoordinate(cylinders[0], models[0])



def visualize(cylinders, models):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    for p, m in zip(cylinders, models):
        x,y,z = findCoordinate(p, m)
        ax.scatter(x, y, z, color = 'black')
        ax.quiver(x, y, z,  m[3], m[4], m[5], color = 'red') 
        p = p.to_array()
        ax.scatter(p[::100,0],p[::100,1],p[::100,2])
    
        

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    
    plt.show()
    return


visualize(cylinders, models)