#!/usr/bin/python3

# coding: utf8
import os, sys
from sys import argv
sys.path.insert(0, os.getcwd()) # adds current directory to python path

from isae.sim_control.gradedSimulation import *
from isae.optim.genAlgParam import *
from isae.tools.footTrajectoryBezier import *

import multiprocessing

print("Number of cpu : ", multiprocessing.cpu_count())

# Loop parameters 
pyb_gui = True
duration = 80

# Trajectory parameters
period = 0.88

offsets = [0,0.5,0.75,0.25]

# Feet trajectories
points = [[-0.3625, 0.0, 0.3680, 0.0, 0.2019, 0.4846, -0.4183, 0.5634], [0.1733, 0.0, 0.176, 0.0, -0.2018, 0.1855, -0.2008, -0.1800]]
footTraj1 = footTrajectoryBezier(points, phaseOffset = offsets[0])
footTraj2 = footTrajectoryBezier(points, phaseOffset = offsets[1])
footTraj3 = footTrajectoryBezier(points, phaseOffset = offsets[2])
footTraj4 = footTrajectoryBezier(points, phaseOffset = offsets[3])

bodyHeights = [1.35] * 4

# Geometry and controller
leg = Leg(1,1)
sols = [False, False, True, True]
#sols = [True, True, False, False]
#sols = [False, False, False, False]
#sols = [True, True, True, True]

#Kp = 8
Kp = 8
#Kd = 0.2
Kd = 0.2

trajs = [footTraj1, footTraj2, footTraj3, footTraj4]
robotController = footTrajController(bodyHeights, leg, sols, trajs, period, Kp, Kd, 3 * np.ones((8, 1)))

dt = 0.001
traj_log = []
for my_t in range(int(period / dt)):
	current_point = []
	for j in range(len(trajs)):
		foot_pos = trajs[j].getPos(dt * my_t / period)
		foot_pos[0,1] -= bodyHeights[j]
		joint_pos = leg.getJointsPos(foot_pos[0], otherSol = sols[j])
		current_point.append(joint_pos[0])
		current_point.append(joint_pos[1])
	traj_log.append(np.array(current_point))

traj_log = np.array(traj_log)
np.save("vertical_main.npy", traj_log)


# Create simulation
walkSim = gradedSimulation()

# Assign parameters to the simulation
walkSim.setLoopParams(pyb_gui, duration, leg)
walkSim.setController(robotController)

walkSim.initializeSim()

# Run sim
walkSim.runSim()

'''
print(walkSim.getFinalDistance())
plt.figure()
walkSim.plotContactPoints()
plt.figure()
walkSim.plotBaseSpeed()
plt.legend()
plt.figure()
walkSim.plotBaseAbsXYSpeed(filters=[50,200,1000])
plt.legend()
plt.figure()
walkSim.plotBasePos()
plt.legend()
#plt.figure()
#walkSim.plotGrades()
#plt.legend()
plt.show()
'''