# -*- coding: utf-8 -*-
"""
Created on Thu Dec 13 09:15:59 2018

@author: bimta
"""
import numpy as np
from open3d import PointCloud, Vector3dVector, estimate_normals, KDTreeSearchParamHybrid, draw_geometries #TODO: Remove last
import time

#Set of Points
las = np.loadtxt('./Data/pole.txt', delimiter = ';',skiprows = 1)
points = las[:,:3]
points = points[np.where(points[:,2] > 2.5)]




# RANSAC Algorithm
class CylRANSAC:
    def __init__(self, points, nr_of_iterations, distance_threshold, min_sample_threshold, tuple_radius_minmax, tuple_theta_phi):
        self.P = points
        self.iterations = nr_of_iterations
        self.t = distance_threshold
        self.min_sample = min_sample_threshold
        self.minradius = tuple_radius_minmax[0]
        self.maxradius = tuple_radius_minmax[1]
        self.normals = self.estimateNormals(points)
        self.dir = self.direction(tuple_theta_phi[0], tuple_theta_phi[1])
        return

    def direction(self, theta, phi):
        '''Return the direction vector of a cylinder defined
        by the spherical coordinates theta and phi.
        '''
        return np.array([np.cos(phi) * np.sin(theta), np.sin(phi) * np.sin(theta), np.cos(theta)])

    def estimateNormals(self, points):
        pcd = PointCloud()
        pcd.points = Vector3dVector(points)
        estimate_normals(pcd, search_param = KDTreeSearchParamHybrid(radius = 0.1, max_nn = 30))
        return np.asarray(pcd.normals)
    
    def sample(self):
        n = 2
        all_indices = np.arange(self.P.shape[0])
        np.random.shuffle(all_indices)
    
        self.indices_S = all_indices[:n]
        self.indices_R = all_indices[n:]
        return 
    
    def fit(self):
        # Selected subset
        S = self.P[self.indices_S,:]
        N = self.normals[self.indices_S,:]

        # Fit model to S        
        # Cylinder axis (normalized)
        a = np.cross(N[0], N[1])
        a = a/np.sqrt(a.dot(a))

        # Check angle
        if np.dot(a, self.dir) < 0.1: # The angle threshold
            # Cylinder cross-section plane normal
            b = np.cross(a, N[0])
            
            #TODO: Make sure that second point is picked so that there can in fact be an intersection - dot > 0.00001
            # point on axis 
            dot = np.dot(b,N[1])
            w = S[1] - S[0]
            s = -np.dot(b, w) / dot
            p = w + s * N[1] + S[0]
            
            #vector from axis to point on surface
            rv = p - S[1]
            
            # Radius
            r = np.sqrt(rv.dot(rv))
            
            # Check radius
            if r > self.minradius and r < self.maxradius:
                return [p, r, a]                      # Cylinder model point on axis, radius, axis
            else:
                return None
        else:
            return None
    
    def evaluateModel(self, model):
        #start = time.process_time()

        # Remaining points
        R = self.P[self.indices_R,:]
        
        # Radius
        r = model[1]
        
        # Point on axis
        c = model[0]
        
        # New basis vectors:
        a1 = model[2] # The axis
        
        #TODO: Improve this
        a2 = np.random.randn(3)
        a2 -= a2.dot(a1) * a1
        a2 /= np.linalg.norm(a2)
        
        a3 = np.cross(a1, a2)
        a3 /= np.linalg.norm(a3)
        
        # Transformation matrix
        b = np.stack((a1,a2,a3), axis = 1)
        
        # Inverse
        binv = np.linalg.inv(b)
        
        # Recentered:
        Rc = R-c
        # Change basis:
        Rbc = np.dot(Rc, binv)
        # Reduce dimensions:
        twod = Rbc[:,1:]
        
        # Distances to axis:
        distances = np.sqrt(twod[:,0]**2+twod[:,1]**2)
        
        # Distance between points and model axis within radius +- threshold
        R_inlier_indices = np.where((distances >= r - self.t) & (distances <= r + self.t))[0]
        inlier_indices = np.hstack((R_inlier_indices, self.indices_S))
        
        if len(inlier_indices) > self.min_sample:
            return inlier_indices
        else: 
            return None

    def run(self):
        nr_inliers = 0
        success = False
        k = 0
        while k <= self.iterations:
            self.sample()
            current_model = self.fit()
            if current_model is not None:
                inlier_indices = self.evaluateModel(current_model)
                if inlier_indices is not None:
                    if len(inlier_indices) > nr_inliers:
                        nr_inliers = len(inlier_indices)
                        inliers = self.P[inlier_indices]
                        model = current_model
                        success = True
            k += 1
            
        if success:
            return [inliers, model]
        else:
            print('Could not find any cylinders')
            return None


MyRANSAC = CylRANSAC(points, 1000, 0.01, 500, (0.01, 0.1), (0,0)) 

start = time.process_time()
result = MyRANSAC.run()
end = time.process_time()        
print("Total {0:.2f} seconds.".format(end - start))


MyRANSAC.sample()
m = MyRANSAC.fit()
i = MyRANSAC.evaluateModel(m)

from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
import matplotlib.pyplot as plt

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