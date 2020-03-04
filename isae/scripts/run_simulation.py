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
#from isae.tools.cameraTool import * 
from isae.control.footTrajControllerV2 import * 
from isae.control.footTrajController import * 

import multiprocessing

print("Number of cpu : ", multiprocessing.cpu_count())

params = [
#0 => -12
0.13193453226003413, 	#length
1.3512355234987758, 	#height
-0.43360866145837307, 	#top_dx
0.08787105222212671, 	#end_dX
0.09274779651763523, 	#end_dy
0.004334391317216024, 	#middle_dx
-0.03784137239537205, 	#middle_dy
0.9681577398305765, 	#onGroundPhase 
0.5236698318512114, 	#preriod
1.8242074368232974, 	#bodyHeight
]

# Loop parameters 
pyb_gui = True
duration = 8

# Trajectory parameters
#period = 1.9
period = 1.2

#offsets = [0.0,0.5,0.0,0.5]
offsets = [0.0,0.5,0.,0.5]

# Feet trajectories
footTraj1 = customTrajectory(params[0], params[1], params[2], params[3], params[4], params[5], params[6], params[7], 0)
footTraj2 = customTrajectory(params[0], params[1], params[2], params[3], params[4], params[5], params[6], params[7], 0.50)
footTraj3 = customTrajectory(params[0], params[1], params[2], params[3], params[4], params[5], params[6], params[7], 0.75)
footTraj4 = customTrajectory(params[0], params[1], params[2], params[3], params[4], params[5], params[6], params[7], 0.25)

# Feet trajectories
footTraj1 = footTrajectory([[-0.5,0],[-0.0,0.9], [0.5,0], [-0.5,0]])
footTraj2 = footTrajectory(         footTraj1.points           )
footTraj3 = footTrajectory([[-0.5,0],[-0.0,0.9], [0.5,0], [-0.5,0]])
footTraj4 = footTrajectory(         footTraj3.points           )

#bodyHeights = 2*[1.3] + 2*[1.3]
bodyHeights = 2*[1.3] + 2*[1.3]
#bodyHeights = [1.7,1.7,1.7,1.7]

# Geometry and controller
leg = Leg(1,1)
sols = [False, False, True, True]
#sols = [True, True, False, False]
#sols = [False, False, False, False]
#sols = [True, True, True, True]

#Kp = 8
Kp = 7
#Kd = 0.2
Kd = 0.2
#Kd = 0.8  # Bezier ex

trajs = [footTraj1, footTraj2, footTraj3, footTraj4]
robotController = footTrajController(bodyHeights, leg, sols, trajs, period, Kp, Kd, 3 * np.ones((8, 1)))

def lerpCyclePhase(phase, xVal=[0.3], yVal=[0.6]):
    phase = phase%1.
    
    xVal.append(1)
    xVal.append(0)
    yVal.append(1)
    yVal.append(0)
    
    for i in range(len(xVal) - 1):
        if phase < xVal[i]:
            return yVal[i-1] + (yVal[i] - yVal[i-1])*(1 - (xVal[i] - phase)/(xVal[i] - xVal[i-1]))



#cyclePhase = lambda x : x
#cyclePhase = lambda x : (x%1.)**0.7

setXVal = []
setYVal = []
#setXVal = [0.187]
#setYVal = [0.714]

#robotController = footTrajController(bodyHeights, leg, sols, trajs, period, Kp, Kd, 3 * np.ones((8, 1)))
robotController = footTrajControllerV2(bodyHeights, leg, sols, trajs, offsets, period, partial(lerpCyclePhase,xVal=setXVal, yVal=setYVal), Kp, Kd, 3 * np.ones((8, 1)))

'''
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
'''

# Create simulation
walkSim = gradedSimulation()

# Assign parameters to the simulation
walkSim.setLoopParams(pyb_gui, duration, leg)
walkSim.setController(robotController)

walkSim.initializeSim()

# Run sim
print(walkSim.revoluteJointIndices)
walkSim.runSim()

print(walkSim.getFinalDistance())
print(walkSim.grades[2])
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
plt.figure()
walkSim.robotController.plotFootTraj()
plt.show()
