# -*- coding: utf-8 -*-
"""
Created on Tue Dec 11 15:20:49 2018

@author: bimta
"""

import pcl

cloud = pcl.load_XYZRGB('./Data/poles.pcd')
print(cloud.size)

def subtractGround(pointcloud):
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
def findCylinder(pointcloud):
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

def findCylinders(pointcloud, nr_of_cylinders):
    results = []
    models = []
    for i in range(nr_of_cylinders):
        #print(pointcloud.size)
        cylinder, model, pointcloud = findCylinder(pointcloud)
        if cylinder.size != 0:
            results.append(cylinder)
            models.append(model)
        else:
            break
    print("You asked me to find {} cylinder(s), I have found {} cylinder(s)".format(nr_of_cylinders, len(results)))
    return results, models

def saveCylinders(cylinders):
    i = 1    
    for cloud in cylinders:
        pcl.save(cloud, 'cyl{}.pcd'.format(i))
        i += 1
    return

pointcloud = subtractGround(cloud)
cylinders, models = findCylinders(pointcloud, 7)
saveCylinders(cylinders)


#TODO: Extract exact position
#TODO: Check with more noisy data


"""

import pcl.pcl_visualization
# from pcl.pcl_registration import icp, gicp, icp_nl
visual = pcl.pcl_visualization.CloudViewing()
visual.ShowColorCloud(remaining)
v = True
while v:
    v=not(visual.WasStopped())
"""