# -*- coding: utf-8 -*-
"""
Created on Sun Dec 16 21:11:42 2018

@author: bimta
"""
from RANSAC_03 import CylRANSAC
import numpy as np
import time

#Set of Points
las = np.loadtxt('C:/Users/bimta/Documents/Arbeit/Malmö_Stad/Git/object_recognition/Data/pole.txt', delimiter = ';',skiprows = 1)
points = las[:,:3]
points = points[np.where(points[:,2] > 3.8)]


MyRANSAC = CylRANSAC(points, 5000, 0.01, 2000, (0.05, 0.2), (0,0)) 

start = time.process_time()
result = MyRANSAC.run()
end = time.process_time()        
print("Done after {0:.2f} seconds.".format(end - start))