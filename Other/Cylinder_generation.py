# -*- coding: utf-8 -*-
"""
Created on Fri Dec 14 14:38:01 2018

@author: bimta
"""
import numpy as np

def rotation_matrix_from_axis_and_angle(u, theta):
    '''Calculate a rotation matrix from an axis and an angle.'''
    
    x = u[0]
    y = u[1]
    z = u[2]
    s = np.sin(theta)
    c = np.cos(theta)
    
    return np.array([[c + x**2 * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
                     [y * x * (1 - c) + z * s, c + y**2 * (1 - c), y * z * (1 - c) - x * s ],
                     [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z**2 * (1 - c) ]])

def make_points_on_a_cylinder(theta, phi, C, r, N):
    '''Make N points on a cylinder defined by the center C, direction defined theta and phi and radius r.
    Also return the direction of the cylinder'''
    
    M = np.dot(rotation_matrix_from_axis_and_angle(np.array([0, 0, 1]), phi),
               rotation_matrix_from_axis_and_angle(np.array([0, 1, 0]), theta))

    x = np.dot(M, np.array([1, 0, 0]))
    y = np.dot(M, np.array([0, 1, 0]))
    z = np.dot(M, np.array([0, 0, 1]))

    delta = np.radians(10)
    t = 0.1

    return [C + r * (np.cos(i * delta) * x + np.sin(i * delta) * y + i * t * z) for i in range(N)], z




