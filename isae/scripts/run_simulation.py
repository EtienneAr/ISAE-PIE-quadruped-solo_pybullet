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


# Loop parameters 
pyb_gui = True
duration = 8

# Trajectory parameters
#period = 1.9
period = 1.2

#offsets = [0.0,0.5,0.0,0.5]
offsets = [0.5,0.,0.5,0.]

# params : bh1, bh2, Kp, Kd, period
#0.96, 1.27, 12.2, 0.52, 0.82   optim1 (pop 10, gen 20)
#1.19, 1.65, 13.3, 0.54, 0.95   optim2 (pop 40, gen 20)
#best_params = [1.3, 1.32, 14.2, 1.1, 2.56]  # for speed ref = [0.2,0]
#1.41570218598
#1.59286447661
#15.3423324557
#3.60968563781
#1.30976605128

# params : triangle summits
t0, t1, t2 = [-0.51, 0.09],[0.52, 1.43],[0.73, 0.07]

# Feet trajectories
footTraj1 = footTrajectory([[-1.14472959,  0.03284315],
       [ 0.46659341,  0.51091852],
       [ 0.68658772,  0.76576489],
       [-1.14472959,  0.03284315]], phaseOffset = offsets[0])
#footTraj1 = footTrajectory([t0,t1,t2,t0], phaseOffset = offsets[0])
footTraj2 = footTrajectory(         footTraj1.points           , phaseOffset = offsets[1])
footTraj3 = footTrajectory([[-1.14472959,  0.03284315],
       [ 0.46659341,  0.51091852],
       [ 0.68658772,  0.76576489],
       [-1.14472959,  0.03284315]], phaseOffset = offsets[2])
footTraj4 = footTrajectory(         footTraj3.points           , phaseOffset = offsets[3])

#bodyHeights = 2*[1.3] + 2*[1.3]
bodyHeights = 2*[0.82] + 2*[0.514]
#bodyHeights = [1.7,1.7,1.7,1.7]

# Geometry and controller
leg = Leg(1,1)
sols = [False, False, True, True]
#sols = [True, True, False, False]
#sols = [False, False, False, False]
#sols = [True, True, True, True]

#Kp = 8
Kp = 10
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
