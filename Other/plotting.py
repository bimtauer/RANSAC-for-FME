# -*- coding: utf-8 -*-
"""
Created on Fri Dec 14 10:33:25 2018

@author: bimta
"""
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
import matplotlib.pyplot as plt
import numpy as np

def plot(G, P):
    fig = plt.figure()
    ax = fig.add_subplot(111, aspect = 'equal', projection='3d')
    
    # All points
    #ax.scatter(P[::100,0], P[::100,1], P[::100,2], color = 'black')
    # Chosen points
    ax.scatter(G[::10,0], G[::10,1], G[::10,2], color = 'orange')
    # Create cubic bounding box to simulate equal aspect ratio
    max_range = np.array([P[:,0].max()-P[:,0].min(), P[:,1].max()-P[:,1].min(), P[:,2].max()-P[:,2].min()]).max()
    Xb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][0].flatten() + 0.5*(P[:,1].max()+P[:,1].min())
    Yb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][1].flatten() + 0.5*(P[:,1].max()+P[:,1].min())
    Zb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][2].flatten() + 0.5*(P[:,2].max()+P[:,2].min())
    # Comment or uncomment following both lines to test the fake bounding box:

    for xb, yb, zb in zip(Xb, Yb, Zb):
       ax.plot([xb], [yb], [zb], 'w')
    
    
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    
    plt.show()
    return

plot(result[0], points)
"""
fig = plt.figure()
ax = fig.add_subplot(111, aspect = 'equal', projection='3d')
    
# All points
ax.scatter(R[::100,0], R[::100,1], R[::100,2], color = 'black')
# Chosen points
ax.scatter(S[:,0], S[:,1], S[:,2], color = 'orange')


# Rebased points:
ax.scatter(Pbc[:,0], Pbc[:,1], Pbc[:,2], color = 'green')

#New basis
for a in b.T:
    ax.quiver(c[0], c[1], c[2], a[0], a[1], a[2], color = 'red') 

for a in b.T:
    ax.quiver(0, 0, 0, a[0], a[1], a[2], color = 'red') 


# Create cubic bounding box to simulate equal aspect ratio
max_range = np.array([P[:,0].max()-P[:,0].min(), P[:,1].max()-P[:,1].min(), P[:,2].max()-P[:,2].min()]).max()
Xb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][0].flatten() + 0.5*(P[:,1].max()+P[:,1].min())
Yb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][1].flatten() + 0.5*(P[:,1].max()+P[:,1].min())
Zb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][2].flatten() + 0.5*(P[:,2].max()+P[:,2].min())
# Comment or uncomment following both lines to test the fake bounding box:


for xb, yb, zb in zip(Xb, Yb, Zb):
   ax.plot([xb], [yb], [zb], 'w')

    

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()

###############################################################################
"""
# Two d plotting

fig = plt.figure()
ax = fig.add_subplot(111, aspect = 'equal')
distances = np.sqrt(twod[:,0]**2+twod[:,1]**2)
        
# Distance between points and model axis within radius +- threshold
inlier_indices = np.where((distances >= 0.10) & (distances <= 0.15))[0]
ax.scatter(twod[:,0], twod[:,1], color = 'red')
ax.scatter(twod[inlier_indices,0], twod[inlier_indices,1], color= 'green')

plt.show()
