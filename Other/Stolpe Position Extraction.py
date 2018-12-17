# -*- coding: utf-8 -*-
"""
Created on Thu Dec 13 15:13:20 2018

@author: bimta
"""

import pcl
import numpy as np

cloud = pcl.load_XYZRGB('./Data/poles.pcd')
print(cloud.size)

"""
pc_1 = pcl.PointCloud()
pc_1.from_array(array[:,:3])
"""
class CylinderExtractor:
    def __init__(self, pointcloud):
        self.pointcloud = self.subtractGround(pointcloud)

    def subtractGround(self, pointcloud):
        seg = pointcloud.make_segmenter_normals(ksearch=50)
        seg.set_optimize_coefficients(True)
        seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
        seg.set_normal_distance_weight(0.1)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_max_iterations(10000)
        seg.set_distance_threshold(0.3)
        indices, model = seg.segment()
        #print(model)
        #cloud_plane = pointcloud.extract(indices, negative=False)
        cloud_without_plane = pointcloud.extract(indices, negative=True)
        return cloud_without_plane
    
    #TODO: filter for cylinder axes with certain angle
    def findCylinder(self, pointcloud):
        seg = pointcloud.make_segmenter_normals(ksearch=50)
        seg.set_optimize_coefficients(True)
        seg.set_model_type(pcl.SACMODEL_CYLINDER)
        seg.set_normal_distance_weight(0.05)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_max_iterations(10000)
        seg.set_distance_threshold(0.3)
        seg.set_radius_limits(0.05, 0.2)
        indices, model = seg.segment()
        #print(model)
        cylinder = pointcloud.extract(indices, negative=False)
        remaining = pointcloud.extract(indices, negative=True)
        return cylinder, model, remaining
    
    def findCylinders(self, pointcloud, nr_of_cylinders):
        results = []
        models = []
        for i in range(nr_of_cylinders):
            #print(pointcloud.size)
            cylinder, model, pointcloud = self.findCylinder(pointcloud)
            if cylinder.size != 0:
                results.append(cylinder)
                models.append(model)
            else:
                break
        print("You asked me to find {} cylinder(s), I have found {} cylinder(s)".format(nr_of_cylinders, len(results)))
        return results, models
    
    def findCoordinate(self, cylinder, model):
        points = cylinder.to_array()
        lowest_point = points[np.where(points[:,2] == points[:,2].min())]
        z = lowest_point[:,2]
        
        #The origin of the cylinder axis
        xp = model[0]
        yp = model[1]
        zp = model[2]
        
        #The direction of the cylinder axis
        xd = model[3]
        yd = model[4]
        zd = model[5]
        
        bla = (z - zp)/zd  
        x = (bla * xd) + xp
        y = (bla * yd) + yp
        return x, y, z
    
    def run(self, nr_of_cyls):
        cylinders, models = self.findCylinders(self.pointcloud, nr_of_cyls)
        
        positions = []
        for cylinder, model in zip(cylinders, models):
            x, y, z = self.findCoordinate(cylinder, model)
            d = model[6]
            point = np.array([x,y,z,d])
            positions.append(point)
        return positions, cylinders
    
Extractor = CylinderExtractor(cloud)  
pos, cyl = Extractor.run(7)