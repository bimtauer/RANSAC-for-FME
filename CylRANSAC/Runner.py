# -*- coding: utf-8 -*-
"""
Created on Sun Dec 16 17:03:52 2018

@author: bimta
"""
from RANSAC import CylRANSAC
import numpy as np

""" Creates an instance of CylRANSAC with the specified parameters and searches
a list of 3d points for the existence of cylinders up to a passed number. Points
input should be an array of 3d coordinates, ideally ground removed
"""
def cylinderSearch(nr, points):
    models = []
    cyl_points = []
    for i in range(nr):
        MyRANSAC = CylRANSAC(points, 5000, 0.01, 2000, (0.05, 0.2), (0,0))
        result = MyRANSAC.run()
        if result is not None:
            models.append(result[2])
            cyl_points.append(points[result[0]])
            points = points[result[1]]
    print('You asked me to find {} cylinder(s), I found {} cylinder(s)'.format(nr, len(cyl_points)))
    return models, cyl_points

#models, cyl_points = cylinderSearch(4, points)










from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
import matplotlib.pyplot as plt

def plot(cyl_points, points):
    P = points

    fig = plt.figure()
    ax = fig.add_subplot(111, aspect = 'equal', projection='3d')

    # Create cubic bounding box to simulate equal aspect ratio
    max_range = np.array([P[:,0].max()-P[:,0].min(), P[:,1].max()-P[:,1].min(), P[:,2].max()-P[:,2].min()]).max()
    Xb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][0].flatten() + 0.5*(P[:,0].max()+P[:,0].min())
    Yb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][1].flatten() + 0.5*(P[:,1].max()+P[:,1].min())
    Zb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][2].flatten() + 0.5*(P[:,2].max()+P[:,2].min())
    # Comment or uncomment following both lines to test the fake bounding box:
    for xb, yb, zb in zip(Xb, Yb, Zb):
        ax.plot([xb], [yb], [zb], 'w')

    # All points
    ax.scatter(P[::100,0], P[::100,1], P[::100,2], color = 'grey', alpha = 0.1)

    for cyl in cyl_points:
        ax.scatter(cyl[::10,0], cyl[::10,1], cyl[::10,2], color = 'green')

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.show()
    return

plot(cyl_points, points)
