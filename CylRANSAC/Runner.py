# -*- coding: utf-8 -*-
"""
Created on Sun Dec 16 17:03:52 2018

@author: bimta
"""
import numpy as np
from ransac_Main import RANSAC


""" Creates an instance of CylRANSAC with the specified parameters and searches
a list of 3d points for the existence of cylinders up to a passed number. Points
input should be an array of 3d coordinates, ideally ground removed
"""
#Specify parameters here
def Search(nr, points, model):
    models = []
    cyl_points = []
    for i in range(nr):
        MyRANSAC = RANSAC(points, 5000, 0.01, 1000)                              # the nr of iterations, the distance treshold for inliers, the minimum nr of points for an object
        MyRANSAC.Model = model        
        result = MyRANSAC.run()
        if result is not None:
            models.append(result[2])
            cyl_points.append(points[result[0]])
            points = points[result[1]]
    print('You asked me to find {} cylinder(s), I found {} cylinder(s)'.format(nr, len(cyl_points)))
    return models, cyl_points, points


