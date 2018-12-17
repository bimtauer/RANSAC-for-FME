# -*- coding: utf-8 -*-
"""
Created on Thu Dec 13 09:15:59 2018

@author: bimta
"""
import numpy as np
from open3d import PointCloud, Vector3dVector, estimate_normals, KDTreeSearchParamHybrid
import time
#from numba import njit


def timeit(method):
    def timed(*args, **kw):
        ts = time.time()
        result = method(*args, **kw)
        te = time.time()

        if 'log_time' in kw:
            name = kw.get('log_name', method.__name__.upper())
            kw['log_time'][name] = int((te - ts) * 1000)
        else:
            print('{}:  {} ms'.format(method.__name__, (te - ts) * 1000))
        return result

    return timed



def constructCylinder(a, N, S):
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
    # Radius normal:
    rv /= r
    a3 = np.cross(a, rv)
    a3 /= np.linalg.norm(a3)
    return p, r, rv, a3


# RANSAC Algorithm
#@jitclass
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
        estimate_normals(pcd, search_param = KDTreeSearchParamHybrid(radius = 0.1, max_nn = 20))
        return np.asarray(pcd.normals)
    
    def sample(self):
        n = 2
        all_indices = np.arange(self.P.shape[0])
        np.random.shuffle(all_indices)
    
        self.indices_S = all_indices[:n]
        self.indices_R = all_indices[n:] #TODO: Remove
        return 
    
    #@timeit
    #@jit(nopython = True)
    def fit(self):
        # Selected subset
        S = self.P[self.indices_S,:]
        N = self.normals[self.indices_S,:]

        # Fit model to S        
        # Cylinder axis (normalized)
        a = np.cross(N[0], N[1])   #f they are parallel, a = [0,0,0] -> division by 0 in next step!!!
        a = a/np.sqrt(a.dot(a))
        
        # Check angle #TODO: Make this faster
        if np.dot(a, self.dir) < -0.98 or np.dot(a, self.dir) > 0.98: # The angle threshold in radians
            p, r, rv, a3 = constructCylinder(a, N, S)
            # Check radius
            if r > self.minradius and r < self.maxradius:
                return [p, r, a, rv, a3]                      # Cylinder model point on axis, radius, axis
            else:
                return None
        else:
            return None
    
    #@timeit
    #@jit(nopython = True)
    def evaluateModel(self, model):
        # Remaining points
        R = self.P #[self.indices_R]    
        # Transformation matrix - drop dimension of main axis
        b = np.stack((model[3],model[4]), axis = 1)    
        # Recentered:
        Rc = R-model[0]
        # Change basis - row * column -> columns of b = rows of b.T = rows of inv(b)
        twod = np.dot(Rc, b)
        # Distances to axis:
        distances = np.sqrt(twod[:,0]**2+twod[:,1]**2)
        # Distance within radius + threshold
        inlier_indices = np.where(distances <= model[1] + self.t)[0]
        outlier_indices = np.where(distances > model[1] + self.t)[0]
        if len(inlier_indices) > self.min_sample:
            return [inlier_indices, outlier_indices]
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
                indices = self.evaluateModel(current_model)
                if indices is not None:
                    if len(indices[0]) > nr_inliers:
                        nr_inliers = len(indices[0])
                        inliers = indices[0]
                        outliers = indices[1]
                        model = current_model
                        success = True
            k += 1
        if success:
            print('Found one cylinder')
            return [inliers, outliers, model]
        else:
            print('Could not find a cylinder')
            return None