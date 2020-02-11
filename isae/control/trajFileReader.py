import numpy as np

def readTrajFromFile(filename):
    return np.load(filename, allow_pickle=True)

# t in [0,T], T period of the trajectory
def getQfromTraj(traj, t):
    i = int(t*1000)%len(traj)
    return traj[i]