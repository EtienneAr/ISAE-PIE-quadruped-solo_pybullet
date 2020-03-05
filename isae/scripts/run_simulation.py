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
from isae.tools.lerpCyclePhasePoly import *

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

# #0 => -17
# 0.13979392750638392, 	#length
# 0.6960755306468621, 	#height
# -0.08652808293856407, 	#top_dx
# 0.10801735343812276, 	#end_dX
# 0.182021280330618, 	#end_dy
# 0.1311407311902144, 	#middle_dx
# 0.13654642698410985, 	#middle_dy
# 0.9400546750111205, 	#onGroundPhase 
# 0.7420926326981498, 	#preriod
# 1.5344035227788582, 	#bodyHeight


# #0 => -19
# 0.3676696428008097, 	#length
# 0.6924094374434756, 	#height
# -0.44255392600420895, 	#top_dx
# 0.06582406119114259, 	#end_dX
# 0.13305972177080083, 	#end_dy
# -0.33606251922069774, 	#middle_dx
# 0.0, 					#middle_dy
# 0.917987629439546, 		#onGroundPhase 
# 0.7307738251919189, 	#preriod
# 1.4801748792118907, 	#bodyHeight



# #0 => -17
# 0.46145736995143083, 	#length
# 1.1088043084730161, 	#height
# -0.767421454708398, 	#top_dx
# 0.15076494691176015, 	#end_dX
# -0.22743789422052113, 	#end_dy
# 0.050391304172427764, 	#middle_dx
# -0.022606724811983114, 	#middle_dy
# 0.9300935097409295, 	#onGroundPhase 
# 0.7434371952924206, 	#preriod
# 1.581872873912653, 	#bodyHeight


# 0.40837721208727384, 	#length
# 0.9549255241301574, 	#height
# -0.47767912333829576, 	#top_dx
# 0.2763327505364216, 	#end_dX
# -0.2642250029947522, 	#end_dy
# -0.09014444197440663, 	#middle_dx
# 0.008976323284597298, 	#middle_dy
# 0.929566372840192, 	#onGroundPhase 
# 0.7633821164171444, 	#preriod
# 1.5696099859696049, 	#bodyHeight


# 0.3052912954491256, 	#length
# 1.0047836625421853, 	#height
# -0.2274264277819842, 	#top_dx
# 0.2544160006389065, 	#end_dX
# -0.17827847838846872, 	#end_dy
# 0.13082316706267982, 	#middle_dx
# 0.14996643787054292, 	#middle_dy
# 0.9061549243444517, 	#onGroundPhase 
# 0.8905525172820173, 	#preriod
# 1.4834742296415058, 	#bodyHeight


# 0.5896923390427692,
# 1.2351411441549947,
# -0.30325369853474826,
# 0.19495336113966782,
# -0.1639554165925736,
# -0.5573880333417938,
# 0.09797553682313326,
# 0.9481789012437343,
# 1.1900672422376402,
# 1.3300417717994568,


# 0.6339712812216678,
# 0.5763344991926671,
# -0.6981066810384158,
# 0.025945223510571658,
# -0.2206678686188413,
# 0.1392758586078262,
# -0.18692625780688682,
# 0.9404547725673897,
# 1.3270760294392112,
# 1.5473986573893712,

# 0.8006669994075996,
# 0.8043329670481421,
# -0.078323670192583,
# 0.138116160023265,
# -0.15535914805882495,
# -0.004626636778647319,
# -0.05041553089667304,
# 0.8849698258781956,
# 1.1601537177339414,
#1.3855528721673105,
]
paramsInstance = [1.3970040759095335,     #BH0
1.3966946651189274,     #BH1
7.1206989066169495,     #Kp
0.9641576669468045,     #Kd
0.800711228688953,      #Period
0.16205778619884972,    #x1
0.7660218028383298,     #x2
0.1955398937242107,     #y1
0.2970654097064856,     #y2
]
# Loop parameters 
pyb_gui = True
duration = 5

# OPTIMIZE BODY HEIGHTS FOR EXAMPLE
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

pointsTraj = [[-0.3625, 0.0, 0.3680, 0.0, 0.2019, 0.4846, -0.2183, 0.5634], [0.1733, 0.0, 0.176, 0.0, -0.2018, 0.1855, -0.2008, -0.1800]]

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
#sols = [True, True, False, False]
#sols = [False, False, False, False]
#sols = [True, True, True, True]

Kp = Kp
Kd = Kd

robotController = footTrajControllerV2(bodyHeights, leg, sols, trajs, offsets, period, partial(lerpCyclePhase,xVal=setXVal, yVal=setYVal), Kp, Kd, 3 * np.ones((8, 1)))

#robotController = footTrajController(bodyHeights, leg, sols, trajs, period, Kp, Kd, 3 * np.ones((8, 1)))
#robotController = footTrajControllerV2(bodyHeights, leg, sols, trajs, offsets, period, partial(lerpCyclePhase,xVal=setXVal, yVal=setYVal), Kp, Kd, 3 * np.ones((8, 1)))
#robotController = footTrajControllerV2(bodyHeights, leg, sols, trajs, offsets, period, fctCycle.lerpCyclePhase_3, Kp, Kd, 3 * np.ones((8, 1)))

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
#plt.figure()
#walkSim.plotGrades()
#plt.legend()
plt.figure()
walkSim.robotController.plotFootTraj(nbPoints = 70)
plt.show()
