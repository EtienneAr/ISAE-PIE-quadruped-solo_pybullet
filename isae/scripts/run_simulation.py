#!/usr/bin/python3

# coding: utf8
import os, sys
from sys import argv
from functools import partial 
sys.path.insert(0, os.getcwd()) # adds current directory to python path

#from isae.sim_control.walkSimulation import *
from isae.sim_control.gradedSimulation import *
from isae.optim.genAlgParam import *
from isae.control.noiser import *
from isae.tools.cameraTool import * 
from isae.control.footTrajControllerV2 import * 
from isae.control.footTrajController import * 
from isae.tools.lerpCyclePhasePoly import *

import multiprocessing

print("Number of cpu : ", multiprocessing.cpu_count())

#########
#param simu
#########
paramsInstance = [1.3795727080519222 ,1.2767326216086354 ,5.542689574090813,
 0.31873028984268814 ,0.8753232056854438 ,0.401656420970068,
 0.8950312650957308, 0.09482759534228254, 0.3111570609668366,
 -0.0663810033069333, 0.4629271898305571, 0.1273156860449515,
 0.4613413391533199, -0.33358014428349286, 0.573564153376901,
 0.1519333210761974, 0.2995929634467611, -0.11509685645212919,
 0.2268761721252515, -0.14311286016085176, -0.15299604582272447,
 [0.04151969, 0.25559237, 0.59991313, 0.36667742]]




# Loop parameters 
pyb_gui = True
duration = 5

# optimize following parameters, offset and traj set before
bh0 = paramsInstance[0]
bh1 = paramsInstance[1]
Kp = paramsInstance[2]
Kd = paramsInstance[3]
period = paramsInstance[4]
x1 = paramsInstance[5]
x2 = paramsInstance[6]
y1 = paramsInstance[7]
y2 = paramsInstance[8]

# Bezier arg points
P0_x = paramsInstance[9]
P1_x = paramsInstance[10]
P2_x = paramsInstance[11]
P2_y = paramsInstance[12]
P3_x = paramsInstance[13]
P3_y = paramsInstance[14]


# Bezier arg derivee
D0_x = paramsInstance[15]
D1_x = paramsInstance[16]
D2_x = paramsInstance[17]
D2_y = paramsInstance[18]
D3_x = paramsInstance[19]
D3_y = paramsInstance[20]
legOffset = paramsInstance[21]


period = period
#offsets = [0.0,0.5,0.5,0.0]
offsets = legOffset 
bodyHeights = 2*[bh0] + 2*[bh1]

#points traj optim en cours
# param : [[x0,y0,x1,y1,x2,y2,x3,y3],[vx0,vy0,vx1,vy1,vx2,vy2,vx3,vy3]
#pointsTraj = [[-0.3625, 0.0, 0.3680, 0.0, 0.2019, 0.4846, -0.2183, 0.5634], [0.1733, 0.0, 0.176, 0.0, -0.2018, 0.1855, -0.2008, -0.1800]]
#2eme points traj (remontee arriere plus rapide)
#pointsTraj = [[-0.3625, 0.0, 0.3680, 0.0, P2_x, P2_y, P3_x, P3_y], [0.1733, 0.0, 0.176, 0.0, -0.2018, 0.1855, -0.2008, -0.1800]]
pointsTraj = [[P0_x, 0.0, P1_x, 0.0, P2_x, P2_y, P3_x, P3_y], [D0_x, 0.0, D1_x, 0.0, D2_x, D2_y, D3_x, D3_y]]

footTraj1 = footTrajectoryBezier(pointsTraj)
footTraj2 = footTrajectoryBezier(pointsTraj)
footTraj3 = footTrajectoryBezier(pointsTraj)
footTraj4 = footTrajectoryBezier(pointsTraj)
trajs = [footTraj1, footTraj2, footTraj3, footTraj4]

def lerpCyclePhase(phase, xVal=[0.5], yVal=[0.5]):
    phase = phase%1.    
    xVal.append(1)
    xVal.append(0)
    yVal.append(1)
    yVal.append(0)    
    for i in range(len(xVal) - 1):
        if phase < xVal[i]:
            return yVal[i-1] + (yVal[i] - yVal[i-1])*(1 - (xVal[i] - phase)/(xVal[i] - xVal[i-1]))

setXVal = [x1,x2]
setYVal = [y1,y2]
leg = Leg(1,1)
sols = [False, False, True, True]
Kp = Kp
Kd = Kd

robotController = footTrajControllerV2(bodyHeights, leg, sols, trajs, offsets, period, partial(lerpCyclePhase,xVal=setXVal, yVal=setYVal), Kp, Kd, 3 * np.ones((8, 1)))

#robotController = footTrajController(bodyHeights, leg, sols, trajs, period, Kp, Kd, 3 * np.ones((8, 1)))
#robotController = footTrajControllerV2(bodyHeights, leg, sols, trajs, offsets, period, partial(lerpCyclePhase,xVal=setXVal, yVal=setYVal), Kp, Kd, 3 * np.ones((8, 1)))
#robotController = footTrajControllerV2(bodyHeights, leg, sols, trajs, offsets, period, fctCycle.lerpCyclePhase_3, Kp, Kd, 3 * np.ones((8, 1)))


cameraTool = cameraTool()
# Turn boolean to True to record Video (only for no parallel sim) 
cameraTool.recordVideo = False
# Main parameters for camera setting
# See cameraTool class for others
# Warning : Create the following folders
cameraTool.fps = 24
cameraTool.heightImage = 512
cameraTool.widthImage = 512
cameraTool.adressImage = 'isae/dataVideo/images/'
cameraTool.adressVideo = 'isae/dataVideo/videos/'


# Create simulation
walkSim = gradedSimulation()
walkSim.setCameraTool(cameraTool)

# Assign parameters to the simulation
walkSim.setLoopParams(pyb_gui, duration, leg)
walkSim.setController(robotController)

walkSim.initializeSim()

# Run sim
walkSim.runSim()

# Sample the trajectory
dt = 0.001
traj_log = []
for my_t in range(int(period / dt)):
	current_point = []
	for j in range(len(trajs)):
		foot_pos = trajs[j].getPos(dt * my_t / period + offsets[j])
		foot_pos[0,1] -= bodyHeights[j]
		joint_pos = leg.getJointsPos(foot_pos[0], otherSol = sols[j])
		current_point.append(joint_pos[0])
		current_point.append(joint_pos[1])
	traj_log.append(current_point)

#print(traj_log)
np.save("test_B_1.npy", traj_log)


print(walkSim.getFinalDistance())
print(walkSim.grades[1])
#plt.figure()
#walkSim.plotContactPoints()
#plt.figure()
#walkSim.plotBaseSpeed()
#plt.legend()
#plt.figure()
#walkSim.plotBaseAbsXYSpeed(filters=[50,200,1000])
#plt.legend()
#plt.figure()
#walkSim.plotBasePos()
#plt.legend()
#plt.figure()
#walkSim.plotGrades()
#plt.legend()
plt.figure()
walkSim.robotController.plotFootTraj(nbPoints = 70)
plt.show()
