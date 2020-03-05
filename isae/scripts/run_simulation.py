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
paramsInstance = [1.3972100690111018,     #BH0
1.3068572993528498,     #BH1
7.31361762238662,       #Kp
0.8325437299822159,     #Kd
0.8808261948626241,     #Period
0.1745214891774621,     #x1
0.8017562659872206,     #x2
0.16310053613432962,    #y1
0.34013176850814875,    #y2
]

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


period = period
offsets = [0.0,0.5,0.5,0.0]
bodyHeights = 2*[bh0] + 2*[bh1]

#points traj optim en cours
# param : [[x0,y0,x1,y1,x2,y2,x3,y3],[vx0,vy0,vx1,vy1,vx2,vy2,vx3,vy3]
#pointsTraj = [[-0.3625, 0.0, 0.3680, 0.0, 0.2019, 0.4846, -0.2183, 0.5634], [0.1733, 0.0, 0.176, 0.0, -0.2018, 0.1855, -0.2008, -0.1800]]
#2eme points traj (remontee arriere plus rapide)
pointsTraj = [[-0.3625, 0.0, 0.3680, 0.0, 0.2019, 0.4846, -0.4183, 0.5634], [0.1733, 0.0, 0.176, 0.0, -0.2018, 0.1855, -0.2008, -0.1800]]

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
