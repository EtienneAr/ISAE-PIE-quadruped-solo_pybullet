#!/usr/bin/python3

# coding: utf8
import os, sys
from sys import argv
sys.path.insert(0, os.getcwd()) # adds current directory to python path

#from isae.sim_control.walkSimulation import *
from isae.sim_control.gradedSimulation import *
from isae.optim.genAlgParam import *
#from isae.gui.gui_client import *

import multiprocessing

print("Number of cpu : ", multiprocessing.cpu_count())

params = [
0.6339712812216678,
0.5763344991926671,
-0.6981066810384158,
0.025945223510571658,
-0.2206678686188413,
0.1392758586078262,
-0.18692625780688682,
0.9404547725673897,
1.3270760294392112,
1.5473986573893712,

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

# Loop parameters 
pyb_gui = True
duration = 80

# Trajectory parameters
#period = 1.9
period = params[8]

#offsets = [0.0,0.5,0.0,0.5]
offsets = [0,0.5,0.75,0.25]

# Feet trajectories

footTraj1 = customTrajectory(params[0], params[1], params[2], params[3], params[4], params[5], params[6], params[7], 0)
footTraj2 = customTrajectory(params[0], params[1], params[2], params[3], params[4], params[5], params[6], params[7], 0.50)
footTraj3 = customTrajectory(params[0], params[1], params[2], params[3], params[4], params[5], params[6], params[7], 0.75)
footTraj4 = customTrajectory(params[0], params[1], params[2], params[3], params[4], params[5], params[6], params[7], 0.25)

bodyHeights = [params[8]] * 4

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