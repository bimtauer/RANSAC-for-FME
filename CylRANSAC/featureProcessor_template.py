import fme
import fmeobjects
import numpy as np
import sys
sys.path.insert(0,r'\\intra.malmo.se\dfs\gemensamt\Projekt\lidar-data (josbir)\FME\Object Recognition\Git\object_recognition')
from CylRANSAC import Runner                                                    #TODO: Fix this ugly mess


class FeatureProcessor(object):
    def __init__(self):
        self.points = []
        pass
    def input(self,feature):
        p = feature.getAllCoordinates()
        self.name = feature.getAttribute('fme_basename')
        self.points.append([p[0][0], p[0][1], p[0][2]])
        pass
        #self.pyoutput(feature)
    def close(self):
        array = np.array(self.points)
        models, cyl_points, remaining = Runner.cylinderSearch(1, array)
        if len(cyl_points) > 0:
            for point in cyl_points[0]:
                feature = fmeobjects.FMEFeature()
                feature.setGeometryType(1)
                feature.setDimension(3)
                feature.addCoordinate(point[0], point[1], point[2])
                feature.setAttribute('z', point[2])
                feature.setAttribute('part_of_cylinder', '1')
                feature.setAttribute('fme_basename',self.name)
                self.pyoutput(feature)
        for point in remaining:
            feature = fmeobjects.FMEFeature()
            feature.setGeometryType(1)
            feature.setDimension(3)
            feature.addCoordinate(point[0], point[1], point[2])
            feature.setAttribute('z', point[2])
            feature.setAttribute('part_of_cylinder', '0')
            self.pyoutput(feature)
        return
