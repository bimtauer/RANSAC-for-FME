# -*- coding: utf-8 -*-
"""
Created on Thu Dec 13 09:15:59 2018

@author: bimta
"""
import numpy as np
from cylinder_fitting import fit
from open3d import PointCloud, Vector3dVector, estimate_normals, KDTreeSearchParamHybrid, draw_geometries #TODO: Remove last
import time

#Set of Points
las = np.loadtxt('./Data/pole.txt', delimiter = ';',skiprows = 1)
points = las[:,:3]
points = points[np.where(points[:,2] > 2.5)]




# RANSAC Algorithm
class CylRANSAC:
    def __init__(self, points, nr_of_iterations, distance_threshold, min_sample_threshold, radius_minmax):
        self.P = points
        self.iterations = nr_of_iterations
        self.t = distance_threshold
        self.min_sample = min_sample_threshold
        self.radius_minmax = radius_minmax
        return
        
    def sample(self):
        n = 3
        all_indices = np.arange(self.P.shape[0])
        np.random.shuffle(all_indices)
    
        self.indices_S = all_indices[:n]
        self.indices_R = all_indices[n:]
        return 
    
    def evaluateModel(self, model):
        #start = time.process_time()

        # Remaining points
        R = self.P[self.indices_R,:]
        
        # Radius
        r = model[2]
        
        # Center
        c = model[1]
        
        # New basis vectors:
        a1 = model[0] # The axis
        
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
        
        nr_inliers = len(inlier_indices)
        #end = time.process_time()        
        #print("Inlier distances {0:.2f} seconds.".format(end - start))
        if nr_inliers > self.min_sample:
            return nr_inliers, model, inlier_indices
        else: 
            return None, None, None
    
    def fit(self):
        # Selected subset
        S = self.P[self.indices_S,:]

        # Fit model to S
        #start = time.process_time()
        model = fit(S)
        #end = time.process_time()        
        #print("Model fitting {0:.2f} seconds.".format(end - start))
        
        # Radius
        r = model[2]
        
        if r > self.radius_minmax[0] and r < self.radius_minmax[1]:
            return self.evaluateModel(model)
            
        else:
            return None, None, None
    

    def run(self):
        best_inliers = 0
        success = False
        k = 0
        while k <= self.iterations:
            self.sample()
            nr_inliers, model, inlier_indices = self.fit()
            if nr_inliers is not None and nr_inliers > best_inliers:
                best_inliers = nr_inliers
                best_model = model
                indices = inlier_indices
                success = True
            k += 1
        if success:
            return best_inliers, best_model, indices
        else:
            print('Could not find any cylinders')
            return None, None, None

start = time.process_time()
MyRANSAC = CylRANSAC(points, 200, 0.05, 500, (0.01, 0.05)) 

start = time.process_time()
nr_inliers, model, indices = MyRANSAC.run()
end = time.process_time()        
print("Total {0:.2f} seconds.".format(end - start))

"""
MyRANSAC.sample()
nr_inliers, model, indices = MyRANSAC.fit()

P = points
G = P[indices]
plot(G, P)

# without - outlier
# Store model with nr of inliers
# If current model is better than previous better store it otherwise discard

# How many samples to pick?
# N = log(1-p)/log(1- (1-e)s)
# e - probability its an outlier
# s - points in sample
# desired probability for a good sample

"""
# Fitting a cylinder
"""
Cylinder is defined by an Axis containing a point C and having unit-length direction W.
The radius of the cylinder is r > 0 



"""