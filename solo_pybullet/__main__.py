# coding: utf8

#####################
#  LOADING MODULES ##
#####################

import time

import pybullet as p  # PyBullet simulator
import numpy as np  # Numpy library

from sys import argv

from isae.control.myController import *
from isae.tools.geometry import *
from isae.tools.trajectory import *
from isae.tools.new_trajectory import *
import isae.optim.grading


fractorTraj, pointsTraj, period, bodyHeight, Kp, Kd, offsets = (None,) * 7

params = list(map(float, argv[3:]))
try:
    bodyHeight = params[0]
    period = params[1]
    fractorTraj = [params[2], 1]
    offsets = [params[3],params[4],params[5],params[6]]
    pointsTraj = [[params[7],params[8]], [params[9],params[10]], [params[11],params[12]]]
    if(len(params) == 15):
        Kp = float(params[13])
        Kd = float(params[14])
    else:
        Kp = 8.0
        Kd = 0.2
        assert(len(params) == 13)
    # for i in range(9, len(params), 2):
    #     pointsTraj += [[params[i], params[i+1]]]
except:
    print(" # Error. Expected parameters are : ")
    print(" # # guiOn rtSimuOn bodyHeight stepPeriod stepLen phaseOffset_1 phaseOffset_2 phaseOffset_3 phaseOffset_4 point0_X point0_Y point1_X point1_Y point2_X point2_Y [Kp Kd]")
    quit()

contTraj = continuousTrajectory([[-1,0],[0,0.8],[1,0], [-1,0]])
sampledTraj = contTraj.toSampledTraj([i/50.0 for i in range(51)])

traj = sampledTraj

<<<<<<< Updated upstream
leg = Leg(1,1)
controller = myController(bodyHeight, leg, traj, period, offsets, Kp, Kd, 3 * np.ones((8, 1)))
grading = isae.optim.grading.grading_RMS()
=======
period = 0.4
trajPoints = [[-0.5,1],[0.5,0.4]]
#traj = pointsTrajectory(trajPoints)
traj = pointsTrajectory([[params[1], params[2]]])
leg = Leg(1,1)
controller = myController(params[0], leg, traj, period, [params[3],params[4],params[5],params[6]], 8., 0.2, 3 * np.ones((8, 1)))

# A AJOUTER EN PARAM
RTF = 0.5
>>>>>>> Stashed changes

# Functions to initialize the simulation and retrieve joints positions/velocities
from .initialization_simulation import configure_simulation, getPosVelJoints

####################
#  INITIALIZATION ##
####################

dt = 0.001  # time step of the simulation
# If True then we will sleep in the main loop to have a 1:1 ratio of (elapsed real time / elapsed time in the
# simulation))
realTimeSimulation = (argv[2] == "True")
enableGUI = (argv[1] == "True")  # enable PyBullet GUI or not
robotId, solo, revoluteJointIndices = configure_simulation(dt, enableGUI)

# Grading
goal_factors = np.vstack([75, 10, 10, 1, 1, 1])
goal_speed = np.vstack([.5, 0, 0, 0, 0, 0])

###############
#  MAIN LOOP ##
###############
leg0_jointsPos = []
leg1_jointsPos = []
leg2_jointsPos = []
leg3_jointsPos = []

<<<<<<< Updated upstream
RTF = 1
total_duration = 10. #s
total_len = int(total_duration / dt)
=======
contact_points = []

>>>>>>> Stashed changes
for i in range(total_len):  # run the simulation during dt * i_max seconds (simulation time)

    # Time at the start of the loop
    if realTimeSimulation:
        t0 = time.clock()

    # Get position and velocity of all joints in PyBullet (free flying base + motors)
    q, qdot = getPosVelJoints(robotId, revoluteJointIndices)

    # Call controller to get torques for all joints
    jointTorques = controller.c(q, qdot, i*dt, dt) # c_walking(q, qdot, dt, solo, i * dt)

    # Set control torques for all joints in PyBullet
    p.setJointMotorControlArray(robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)

    # Compute one step of simulation
    p.stepSimulation()

    grading.grade(q[:6], qdot[:6], goal_speed, goal_factors, dt)

    # Sleep to get a real time simulation
    if realTimeSimulation:
        t_step = time.clock() - t0
        t_sleep = dt/RTF - t_step
        if t_sleep > 0:
            time.sleep(t_sleep)

    # Store leg1 joint values
    leg0_jointsPos.append([q[7], q[8]])
    leg1_jointsPos.append([q[9], q[10]])
    leg2_jointsPos.append([q[11], q[12]])
    leg3_jointsPos.append([q[13], q[14]])

    contacts = p.getContactPoints(bodyA = 0)
    for point in contacts : 
        contact_points.append([point[5][0], point[5][1]])



leg0_footpos = map(lambda joints_pos : leg.getFootPos(joints_pos), leg0_jointsPos)
leg1_footpos = map(lambda joints_pos : leg.getFootPos(joints_pos), leg1_jointsPos)
leg2_footpos = map(lambda joints_pos : leg.getFootPos(joints_pos), leg2_jointsPos)
leg3_footpos = map(lambda joints_pos : leg.getFootPos(joints_pos), leg3_jointsPos)

#print grading
print("Result :")
print(str(grading.getGrade()))

# Shut down the PyBullet client
p.disconnect()

<<<<<<< Updated upstream
plt.figure()
=======
#traj.plotTrajectory()

fig1 = plt.figure()
>>>>>>> Stashed changes
plt.plot([p[0] + 0.5 for p in leg0_footpos], [p[1] - 1 for p in leg0_footpos])
plt.plot([p[0] + 0.5 for p in leg1_footpos], [p[1] + 1 for p in leg1_footpos])
plt.plot([p[0] - 0.5 for p in leg2_footpos], [p[1] + 1 for p in leg2_footpos])
plt.plot([p[0] - 0.5 for p in leg3_footpos], [p[1] - 1 for p in leg3_footpos])

<<<<<<< Updated upstream
plt.figure()
traj.plot()
sampledTraj.plot()
plt.show()
=======
fig2 = plt.figure()
plt.scatter([p[0] for p in contact_points], [p[1] for p in contact_points])

plt.show()
>>>>>>> Stashed changes
