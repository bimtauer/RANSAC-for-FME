import fme
import fmeobjects
import numpy as np
# Template Function interface:
# When using this function, make sure its name is set as the value of
# the 'Class or Function to Process Features' transformer parameter
def processFeature(feature):
    pass

# Template Class Interface:
# When using this class, make sure its name is set as the value of
# the 'Class or Function to Process Features' transformer parameter
class FeatureProcessor(object):
    def __init__(self):
        self.points = []
        pass
    def input(self,feature):
        p = feature.getAllCoordinates()
        self.points.append([p[0][0], p[0][1], p[0][2]])
        pass
        #self.pyoutput(feature)
    def close(self):
        array = np.array(self.points)
        print(array)
        pass
