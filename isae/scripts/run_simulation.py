# coding: utf8
import os, sys
from sys import argv
sys.path.insert(0, os.getcwd()) # adds current directory to python path

#from isae.sim_control.walkSimulation import *
from isae.sim_control.gradedSimulation import *
from isae.optim.genAlgParam import *
#from isae.gui.gui_client import *

# Loop parameters 
pyb_gui = True
duration = 15

# Trajectory parameters
#period = 1.9
period = 0.95

#offsets = [0.0,0.5,0.0,0.5]
offsets = [0.5,0.,0.5,0.]

# params : bh1, bh2, Kp, Kd, period
#0.96, 1.27, 12.2, 0.52, 0.82   optim1 (pop 10, gen 20)
#1.19, 1.65, 13.3, 0.54, 0.95   optim2 (pop 40, gen 20)

#
# params : triangle summits
t0, t1, t2 = [-0.51, 0.09],[0.52, 1.43],[0.73, 0.07]

# Feet trajectories
footTraj1 = footTrajectory([[-0.6,0],[-0.0,0.9], [0.6,0], [-0.6,0]], phaseOffset = offsets[0])
#footTraj1 = footTrajectory([t0,t1,t2,t0], phaseOffset = offsets[0])
footTraj2 = footTrajectory(         footTraj1.points           , phaseOffset = offsets[1])
footTraj3 = footTrajectory([t0,t1,t2,t0], phaseOffset = offsets[2])
footTraj4 = footTrajectory(         footTraj3.points           , phaseOffset = offsets[3])

#bodyHeights = 2*[1.3] + 2*[1.3]
bodyHeights = 2*[1.19] + 2*[1.65]
#bodyHeights = [1.7,1.7,1.7,1.7]

# Geometry and controller
leg = Leg(1,1)
sols = [False, False, True, True]
#sols = [True, True, False, False]
#sols = [False, False, False, False]
#sols = [True, True, True, True]

#Kp = 8
Kp = 13.3
#Kd = 0.2
Kd = 0.54

best_param = [0,0,0,0]
best_dist = 0

walkSim = gradedSimulation()

trajs = [footTraj1, footTraj2, footTraj3, footTraj4]
robotController = footTrajController(bodyHeights, leg, sols, trajs, period, Kp, Kd, 3 * np.ones((8, 1)))

# Assign parameters to the simulation
walkSim.setLoopParams(pyb_gui, duration)
walkSim.setController(robotController)
walkSim.setTrajectoryParams(leg, period, trajs, bodyHeights)

walkSim.initializeSim()

# Run sim
walkSim.runSim()

print(walkSim.getFinalDistance())
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
walkSim.plotGrades()
plt.legend()
plt.show()
