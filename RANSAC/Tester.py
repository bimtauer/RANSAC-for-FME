# -*- coding: utf-8 -*-
"""
Created on Sat Dec 22 20:03:35 2018

@author: bimta
"""
from open3d import PointCloud, draw_geometries, Vector3dVector, estimate_normals, KDTreeSearchParamHybrid
import numpy as np
points = np.loadtxt(r'../Data/pole2.txt', skiprows=1, delimiter = ';')[:,:3]

#MyRANSAC.Model = CylinderModel((0,0), (0.05, 0.3))                      #Theta and Phi, Min Max radius of the cylinders
#PlaneModel()

from Runner import Search
from geometry import *

Result = Search(1, points, CylinderModel((0,0), (0.05, 0.1)))

Result = Search(1, points, PlaneModel())

resultPoints = Result[1][0]
#resultPoints = np.vstack((Result[1][0], Result[1][1]))

# For visualization:
pcd = PointCloud()
pcd.points = Vector3dVector(Result[2])
pcd2 = PointCloud()
pcd2.points = Vector3dVector(resultPoints)
pcd2.paint_uniform_color([0.1, 0.1, 0.1])

estimate_normals(pcd2, search_param = KDTreeSearchParamHybrid(radius = 0.1, max_nn = 20))
draw_geometries([pcd2])




#########
pcd = PointCloud()
pcd.points = Vector3dVector(points)
draw_geometries([pcd])