import matplotlib.pyplot as plt
from math import sqrt
import numpy as np

# Defines 2 ways of representing a trajectory.
#   - continuous trajectory : geometrical description, given by a set of points
#   - sampled trajectory : description of a cycle of duration T sampled at dt, also as a (larger) set of points

class continuousTrajectory:
    def __init__(self, points, phaseOffset=0):
        # points : list of size 2 lists
        # ex for a triang. traj.: points = [[-1,0] , [0,1] , [1,0]]
        self.points = np.array(points)
        self.phaseOffset = phaseOffset
        self.segLengths = (np.diff(self.points[:,0])**2 + np.diff(self.points[:,1])**2)**.5 # segment lengths, size n
        self.cumulLength = np.concatenate(([0],np.cumsum(self.segLengths))) # cumulated lengths, size n

    # replaced by more generic getPos
    """
    def getPos(self, phase):
        #Contact with the ground
        phase %= 1

        if(phase < 0.5):
            x_pos = 0.5 - phase*2.
            y_pos = 0

            return [x_pos, y_pos]

        #Navigates between points
        sub_phase = 2. * (phase - 0.5)

        current_point = min(int(sub_phase * (len(self.points)-1)), len(self.points)-2)
        sub_phase_for_point = sub_phase  * (len(self.points)-1) - 1.0 * current_point

        prev_point = self.points[current_point]
        next_point = self.points[current_point+1]

        x_pos = prev_point[0] + (next_point[0]-prev_point[0]) * sub_phase_for_point
        y_pos = prev_point[1] + (next_point[1]-prev_point[1]) * sub_phase_for_point

        return [x_pos/2., y_pos]
    """

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
    
    # deprecated
    """
    def toSampledTraj(self, cpos_list = [i/9.0 for i in range(10)]):
        self.points = np.array(self.points)
        segLengths = (np.diff(self.points[:,0])**2 + np.diff(self.points[:,1])**2)**.5 # segment lengths, size n
        cumulLength = np.cumsum(segLengths) # cumulated lengths, size n
        cpos_list = cumulLength.max()*np.array(cpos_list) # scale cpos_list to match path length

        curr_point = self.points[0]
        curr_seg = 0

        sampledTraj = [self.points[0]]

        for i in range(len(cpos_list) - 2):
            if cpos_list[i + 1] < cumulLength[curr_seg]:
                direction = (self.points[curr_seg + 1] - self.points[curr_seg])/(np.linalg.norm(self.points[curr_seg + 1] - self.points[curr_seg]))
                mag = (cpos_list[i + 1] - cpos_list[i])

            else:
                curr_seg += 1
                curr_point = self.points[curr_seg]

                direction = (self.points[curr_seg+1] - self.points[curr_seg])/(np.linalg.norm(self.points[curr_seg + 1] - self.points[curr_seg]))
                mag = (cpos_list[i + 1] - cumulLength[curr_seg - 1])

            curr_point = curr_point + mag*direction
            sampledTraj.append(curr_point)
        
        return continuousTrajectory(sampledTraj)
    """

    def plot(self):
        plt.plot([p[0] for p in self.points], [p[1] for p in self.points],'-o')

# deprecated
"""
class sampledTrajectory:
    def __init__(self, points):
        self.points = points

    def getPos(self, ind):
        ind %= len(self.points)
        ind = int(ind)
        return self.points[ind]

    def toSampledTraj(self, cpos_list = [i/10.0 for i in range(10)]):
        contTraj = continuousTrajectory(self.points)
        return contTraj.toSampledTraj(cpos_list)

    def plot(self):
        plt.scatter([p[0] for p in self.points], [p[1] for p in self.points],'-o')
"""