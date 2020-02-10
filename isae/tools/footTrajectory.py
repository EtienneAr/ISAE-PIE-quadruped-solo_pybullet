#import matplotlib.pyplot as plt
from math import sqrt
import numpy as np

# Defines a trajectory with a set of points and an optional phase offset

class pointsTrajectory:
    def __init__(self, points, factor = [1,1], phaseOffset= 0, onGroundPhase=0.5):
        self.factor = factor
        self.points = [[-1, 0]] + points + [[1, 0]]
        self.phaseOffset = phaseOffset
        self.onGroundPhase = onGroundPhase

    def getPos(self, phase, factor = None):
        factorTotal = self.factor
        if(factor != None):
            for i in range(len(factor)):
                factorTotal *= factor[i]
        
        #Contact with the ground
        phase += self.phaseOffset
        phase %= 1

        if(phase < self.onGroundPhase):
            x_pos = factorTotal[0] * (self.onGroundPhase - phase*2.)
            y_pos = 0

            return np.array([[x_pos, y_pos]])
    
        #Navigates between points
        sub_phase = (phase - self.onGroundPhase) / (1. - self.onGroundPhase)
        
        current_point = min(int(sub_phase * (len(self.points)-1)), len(self.points)-2)
        sub_phase_for_point = sub_phase  * (len(self.points)-1) - 1.0 * current_point

        prev_point = self.points[current_point]
        next_point = self.points[current_point+1]

        x_pos = prev_point[0] + (next_point[0]-prev_point[0]) * sub_phase_for_point
        y_pos = prev_point[1] + (next_point[1]-prev_point[1]) * sub_phase_for_point
        
        return np.array([[x_pos * factorTotal[0] /2., y_pos * factorTotal[1]]])

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