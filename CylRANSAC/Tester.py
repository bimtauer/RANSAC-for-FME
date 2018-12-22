# -*- coding: utf-8 -*-
"""
Created on Sat Dec 22 20:03:35 2018

@author: bimta
"""
from open3d import PointCloud, draw_geometries, Vector3dVector
import numpy as np
points = np.loadtxt(r'../Data/pole2.txt', skiprows=1, delimiter = ';')[:,:3]

#MyRANSAC.Model = CylinderModel((0,0), (0.05, 0.3))                      #Theta and Phi, Min Max radius of the cylinders
#PlaneModel()

from Runner import Search
from geometry import *

Result = Search(2, points, PlaneModel())#CylinderModel((0,0), (0.05, 0.3)))

resultPoints = np.vstack((Result[1][0], Result[1][1]))

# For visualization:
pcd = PointCloud()
pcd.points = Vector3dVector(points)
draw_geometries([pcd])