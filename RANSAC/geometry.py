import numpy as np

class CylinderModel():
    def __init__(self, distance_threshold, tuple_radius_minmax, tuple_theta_phi):  #TODO: change theta phi behaviour
        self.n = 2
        self.name = 'cylinder'
        self.t = distance_threshold
        self.minradius = tuple_radius_minmax[0]
        self.maxradius = tuple_radius_minmax[1]
        self.dir = self.direction(tuple_theta_phi[0], tuple_theta_phi[1])

    def direction(self, theta, phi):
        #TODO: Copied this code - give credit
        '''Return the direction vector of a cylinder defined
        by the spherical coordinates theta and phi.
        '''
        return np.array([np.cos(phi) * np.sin(theta), np.sin(phi) * np.sin(theta), np.cos(theta)])

    def fit(self, S, N):
        # Fit model to S
        # Cylinder axis (normalized)
        a = np.cross(N[0], N[1])  #f they are parallel, a = [0,0,0] -> division by 0 in next step!!!
        a /= np.sqrt(a.dot(a))

        # Check angle #TODO: Make this faster
        if np.dot(a, self.dir) < -0.98 or np.dot(a, self.dir) > 0.98: # The angle threshold in radians
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
            # Check radius
            if r > self.minradius and r < self.maxradius:
                return [p, r, a, rv, a3]                      # Cylinder model point on axis, radius, axis
            else:
                return None
        else:
            return None

    def evaluate(self, model, R, min_sample):
        # Transformation matrix - drop dimension of main axis
        b = np.stack((model[3],model[4]), axis = 1)
        # Recentered:
        Rc = R-model[0]
        # Change basis - row * column -> columns of b = rows of b.T = rows of inv(b)
        twod = np.dot(Rc, b)
        # Distances to axis:
        distances = np.sqrt(twod[:,0]**2+twod[:,1]**2)
        # Distance within radius + threshold
        inlier_indices = np.where((distances <= model[1] + self.t) & (distances >= model[1] - self.t))[0]
        outlier_indices = np.where((distances > model[1] + self.t) | (distances < model[1] - self.t))[0]
        if len(inlier_indices) > min_sample:
            return [inlier_indices, outlier_indices]
        else:
            return None

"""TODO: implement this way
For a plane, {p1, p2, p3} constitutes a minimal set
when not taking into account the normals in the points. To
confirm the plausibility of the generated plane, the deviation
of the plane’s normal from n1, n2, n3 is determined and the
candidate plane is accepted only if all deviations are less than
the predefined angle α.
""""
class PlaneModel():
    def __init__(self, distance_threshold):
        self.n = 1
        self.name = 'plane'
        self.t = distance_threshold
        pass

    def fit(self, S, N):
        #N is the normal vector of our plane model, S is a point on it, no fitting needed
        N = N/np.sqrt(N.ravel().dot(N.ravel()))
        return [S, N]

    def evaluate(self, model, R, min_sample):
        #Profject every point on plane normal vector thereby getting distances to plane.
        distances = np.dot((R - model[0]), model[1].T)
        inlier_indices = np.where((distances <= self.t) & (distances >= -self.t))[0]
        outlier_indices = np.where((distances > self.t) | (distances < -self.t))[0]
        if len(inlier_indices) > min_sample:
            return [inlier_indices, outlier_indices]
        else:
            return None
