# -*- coding: utf-8 -*-
"""
Created on Fri Dec 14 21:34:52 2018

@author: bimta
"""

import numpy as np
from open3d import *

pcd = read_point_cloud("./Data/poles.pcd")
draw_geometries([pcd])
estimate_normals(pcd, search_param = KDTreeSearchParamHybrid(
        radius = 0.1, max_nn = 30))

normals = np.asarray(pcd.normals)

draw_geometries([normals])