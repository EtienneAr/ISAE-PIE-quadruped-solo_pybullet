# coding: utf8
import os, sys
from sys import argv
sys.path.insert(0, os.getcwd()) # adds current directory to python path

#from isae.sim_control.walkSimulation import *
from isae.sim_control.gradedSimulation import *
#from isae.gui.gui_client import *

# Loop parameters 
pyb_gui = False
duration = 10

# Trajectory parameters
period = 1.6
offsets = [0.0,0.5,0.0,0.5]
#offsets = [0.3,0.3,0.,0.]

# Feet trajectories
footTraj1 = footTrajectory([[-0.5,0],[-0.0,0.8], [0.5,0], [-0.5,0]], phaseOffset = offsets[0])
footTraj2 = footTrajectory(         footTraj1.points           , phaseOffset = offsets[1])
footTraj3 = footTrajectory([[-0.5,0],[0.0,0.8], [0.5,0], [-0.5,0]], phaseOffset = offsets[2])
footTraj4 = footTrajectory(         footTraj3.points           , phaseOffset = offsets[3])

bodyHeights = 2*[1.3] + 2*[1.3]
#bodyHeights = [1.7,1.7,1.7,1.7]

# Geometry and controller
leg = Leg(1,1)
#sols = [False, False, True, True]
sols = [True, True, False, False]
#sols = [False, False, False, False]
#sols = [True, True, True, True]
Kp = 8
Kd = 0.2

best_param = [0,0,0,0]
best_dist = 0

walkSim = gradedSimulation()

trajs = [footTraj1, footTraj2, footTraj3, footTraj4]
robotController = footTrajController(bodyHeights, leg, sols, trajs, period, Kp, Kd, 3 * np.ones((8, 1)))

# Assign parameters to the simulation
walkSim.setLoopParams(True, 10)
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
walkSim.plotBasePos()
plt.legend()
plt.show()

"""
for k in range(1):
    # Create sim instance
    walkSim = walkSimulation()

    random1 = np.random.random()
    random2 = np.random.random()
    random3 = np.random.random()
    random4 = np.random.random()

    randomFootTraj1 = footTrajectory([[-random1,0],[random2 - random3*(random2+random1),random4], [random2,0], [-random1,0]], phaseOffset = offsets[0])
    randomFootTraj2 = footTrajectory(         randomFootTraj1.points           , phaseOffset = offsets[1])
    randomFootTraj3 = footTrajectory(         randomFootTraj1.points           , phaseOffset = offsets[2])
    randomFootTraj4 = footTrajectory(         randomFootTraj1.points           , phaseOffset = offsets[3])

    #trajs = 4*[footTraj1]
    #trajs = [footTraj1, footTraj2, footTraj3, footTraj4]
    trajs = [randomFootTraj1, randomFootTraj2, randomFootTraj3, randomFootTraj4]

    # Assign parameters to the simulation
    walkSim.setLoopParams(pyb_gui, duration)
    walkSim.setControllerParams(leg, sols, Kp, Kd)
    walkSim.setTrajectoryParams(period, trajs, bodyHeights)
    walkSim.initializeSim()

    # Run sim
    walkSim.runSim()

    dist = walkSim.getFinalDistance()
    if dist>best_dist : 
        best_param = [random1, random2, random3, random4]
        best_dist = dist

print(best_param)

walkSim = walkSimulation()

BestFootTraj1 = footTrajectory([[-best_param[0],0],[best_param[1] - best_param[2]*(best_param[1]+best_param[0]),best_param[3]], [best_param[2],0], [-best_param[0],0]], phaseOffset = offsets[0])
BestFootTraj2 = footTrajectory(         BestFootTraj1.points           , phaseOffset = offsets[1])
BestFootTraj3 = footTrajectory(         BestFootTraj1.points           , phaseOffset = offsets[2])
BestFootTraj4 = footTrajectory(         BestFootTraj1.points           , phaseOffset = offsets[3])

trajs = [BestFootTraj1, BestFootTraj2, BestFootTraj3, BestFootTraj4]

# Assign parameters to the simulation
walkSim.setLoopParams(True, 10)
walkSim.setControllerParams(leg, sols, Kp, Kd)
walkSim.setTrajectoryParams(period, trajs, bodyHeights)
walkSim.initializeSim()

# Run sim
walkSim.runSim()

print(walkSim.getFinalDistance())
plt.figure()
walkSim.plotBaseSpeed()
plt.legend()
plt.figure()
walkSim.plotBasePos()
plt.legend()
plt.show()
"""
