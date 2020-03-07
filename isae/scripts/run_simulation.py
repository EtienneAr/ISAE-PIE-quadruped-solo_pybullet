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



paramValue = [
#6 => -66
1.1293904916054385, 	#period
0.5478720740686913, 	#height
0.7078582387894816, 	#bezierPoint
0.9556902212883827, 	#bezierPoint
0.0016396149767283763, 	#bezierPoint
-0.5318452598941055, 	#bezierPoint
0.71971679771012, 	#bezierPoint
-0.39564527741863187, 	#bezierPoint
0.8975717756008055, 	#bezierPoint
0.41906450871473666, 	#bezierPoint
-0.5851068122549943, 	#bezierPoint
-0.17640962111118585, 	#bezierPoint
0.8281301289419223, 	#bezierPoint
-0.873761598039754, 	#bezierPoint


]

period = paramValue[0] #paramValue[8]

bodyHeights = [paramValue[1]] * 4

points = [[paramValue[2], 0.0, paramValue[3], 0.0, paramValue[4], paramValue[5], paramValue[6], paramValue[7]], [paramValue[8], 0.0, paramValue[9], 0.0, paramValue[10], paramValue[11], paramValue[12], paramValue[13]]]


footTraj1 = footTrajectoryBezier(points, phaseOffset = 0)
footTraj2 = footTrajectoryBezier(points, phaseOffset = 0.5)
footTraj3 = footTrajectoryBezier(points, phaseOffset = 0.75)
footTraj4 = footTrajectoryBezier(points, phaseOffset = 0.25)
trajs = [footTraj1, footTraj2, footTraj3, footTraj4]

Kp = 6
Kd = 0.01

# Geometry and controller
leg = Leg(1,1)
sols = [False, False, True, True]

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
np.save("bezier_3pattes.npy", traj_log)


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