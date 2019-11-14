import matplotlib.pyplot as plt
from math import sqrt
import numpy as np

# Defines 2 ways of representing a trajectory.
#   - continuous trajectory : geometrical description, given by a set of points
#   - sampled trajectory : description of a cycle of duration T sampled at dt, also as a (larger) set of points

class footTrajectory:
    def __init__(self, points, phaseOffset=0):
        # points : list of size 2 lists
        # phaseOffset : float in [0,1[, starting point on the trajectory
        # ex for a triang. traj.: points = [[-1,0] , [0,1] , [1,0]]
        self.points = np.array(points)
        self.phaseOffset = phaseOffset
        self.segLengths = (np.diff(self.points[:,0])**2 + np.diff(self.points[:,1])**2)**.5 # segment lengths, size n
        self.cumulLength = np.concatenate(([0],np.cumsum(self.segLengths))) # cumulated lengths, size n

    def getPos(self, phase): # phase in [0,1] over the path
        phase += self.phaseOffset
        phase %= 1
        phase *= self.cumulLength.max()
        segIndex = np.argwhere(self.cumulLength < phase)
        if len(segIndex > 1):
            segIndex = segIndex[-1]
        else:
            segIndex = segIndex[0]

        prevPoint = self.points[segIndex]
        prevPhase = self.cumulLength[segIndex]

        direction = (self.points[segIndex + 1] - self.points[segIndex])/(np.linalg.norm(self.points[segIndex + 1] - self.points[segIndex]))
        mag = phase - prevPhase

        return prevPoint + mag*direction

    def plot(self):
        plt.plot([p[0] for p in self.points], [p[1] for p in self.points],'-o')
