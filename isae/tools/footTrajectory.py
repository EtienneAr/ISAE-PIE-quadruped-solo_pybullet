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

class customTrajectory:
    def trajLen(self, traj):
        length = 0
        for i in range(len(traj) - 1):
            length += sqrt((traj[i][0] - traj[i+1][0])**2 + (traj[i][1] - traj[i+1][1])**2)

        return 1.*length

    def subPhases(self, traj):
        totalLen = self.trajLen(traj)
        res = []
        for i in range(1, len(traj)):
            res.append(self.trajLen(traj[:i]) / totalLen)
        res.append(1.0)
        return res

    def __init__(self, length, height, top_dx, end_dX, end_dy, middle_dx, middle_dy, onGroundPhase, phaseOffset):
        self.onGroundPhase = onGroundPhase
        self.phaseOffset = phaseOffset

        self.pointsOnGround = [[-length, 0], [middle_dx, middle_dy], [length, 0]] 
        self.pointsTraj = [[length, 0], [length + end_dX, end_dy], [top_dx, height], [-length - end_dX, end_dy], [-length, 0]]

        self.subPhasesOnGround = self.subPhases(self.pointsOnGround)
        self.subPhasesTraj = self.subPhases(self.pointsTraj)

    def getPos(self, phase):
        #Contact with the ground
        phase += self.phaseOffset
        phase %= 1

        subPhase = None
        subPhasesList = None
        pointsList = None

        if(phase < self.onGroundPhase):
            subPhase = phase / self.onGroundPhase
            pointsList = self.pointsOnGround
            subPhasesList = self.subPhasesOnGround
        else:
            subPhase = (phase - self.onGroundPhase) / (1. - self.onGroundPhase)
            pointsList = self.pointsTraj
            subPhasesList = self.subPhasesTraj
    
        #Navigates between points
        i = 0
        while subPhase > subPhasesList[i+1]:
            i += 1

        prev_point = pointsList[i]
        next_point = pointsList[i+1]

        sub_phase_for_point = (subPhase - subPhasesList[i]) / (subPhasesList[i+1] - subPhasesList[i])

        x_pos = prev_point[0] + (next_point[0]-prev_point[0]) * sub_phase_for_point
        y_pos = prev_point[1] + (next_point[1]-prev_point[1]) * sub_phase_for_point
        
        return np.array([[x_pos, y_pos]])
