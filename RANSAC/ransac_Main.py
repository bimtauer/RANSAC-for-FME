# -*- coding: utf-8 -*-
"""
Created on Thu Dec 13 09:15:59 2018

@author: bimta
"""
import numpy as np
from open3d import PointCloud, Vector3dVector, estimate_normals, KDTreeSearchParamHybrid


class RANSAC:
    def __init__(self, points, nr_of_iterations, distance_threshold, min_sample_threshold):
        self.P = points
        self.iterations = nr_of_iterations
        self.t = distance_threshold
        self.min_sample = min_sample_threshold
        self.normals = self.estimateNormals(points)
        self.Model = ""
        return

    def estimateNormals(self, points):
        pcd = PointCloud()
        pcd.points = Vector3dVector(points)
        estimate_normals(pcd, search_param = KDTreeSearchParamHybrid(radius = 0.1, max_nn = 20))
        return np.asarray(pcd.normals)

    def sample(self):
        try:
            n = self.Model.n
        except AttributeError:
            raise AttributeError("It looks like you have not specified a shape to search for.")
           
        all_indices = np.arange(self.P.shape[0])
        np.random.shuffle(all_indices)
        self.indices_S = all_indices[:n]
        return

    def run(self):
        nr_inliers = 0
        success = False
        k = 0
        while k <= self.iterations:
            self.sample()
            S = self.P[self.indices_S,:]
            N = self.normals[self.indices_S,:]
            current_model = self.Model.fit(S, N)
            if current_model is not None:
                indices = self.Model.evaluate(current_model, self.P, self.t, self.min_sample)
                if indices is not None:
                    if len(indices[0]) > nr_inliers:
                        nr_inliers = len(indices[0])
                        inliers = indices[0]
                        outliers = indices[1]
                        model = current_model
                        success = True
            k += 1
        if success:
            print('Found a {}'.format(self.Model.name))
            return [inliers, outliers, model]
        else:
            print('Could not find a {}'.format(self.Model.name))
            return None
