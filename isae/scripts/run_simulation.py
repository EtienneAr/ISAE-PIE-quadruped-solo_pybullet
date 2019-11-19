# coding: utf8
import os, sys
from sys import argv
sys.path.insert(0, os.getcwd()) # adds current directory to python path

from isae.sim_control.walkSimulation import *

# Create sim instance
walkSim = walkSimulation()

# Loop parameters 
pyb_gui = True
duration = 20.

# Trajectory parameters
period = 3

offsets = [0,0.5,0.5,0]

# Feet trajectories
footTraj1 = footTrajectory([[-0.5,0],[0.,.5], [0.5,0], [-0.5,0]], phaseOffset = offsets[0])
footTraj2 = footTrajectory(         footTraj1.points           , phaseOffset = offsets[1])
footTraj3 = footTrajectory([[-0.5,0],[0.,.5], [0.5,0], [-0.5,0]], phaseOffset = offsets[2])
footTraj4 = footTrajectory(         footTraj3.points           , phaseOffset = offsets[3])

#trajs = 4*[footTraj1]
trajs = [footTraj1, footTraj2, footTraj3, footTraj4]

bodyHeights = 4*[1.4]
#bodyHeights = [1.7,1.7,1.7,1.7]

# Geometry and controller
leg = Leg(1,1)
Kp = 8
Kd = 0.2

# Assign parameters to the simulation
walkSim.setLoopParams(pyb_gui, duration)
walkSim.setControllerParams(leg, Kp, Kd)
walkSim.setTrajectoryParams(period, trajs, bodyHeights)

# Run sim
walkSim.runSim()

#walkSim.plotFeetAvgPos()

walkSim.plotContactPoints()
walkSim.plotBasePos()
plt.show()

