# coding: utf8

#####################
#  LOADING MODULES ##
#####################

import time

import pybullet as p  # PyBullet simulator

# from .controller import c_walking  # Controller functions

from sys import path, argv


from isae.myController import *
from isae.geometry import *
from isae.trajectory import *

#arguments gui, rtSimu, bodyHeight, trajPoint1, trajPoint2, phaseOff1, phaseOff2, phaseOff3, phaseOff4
params = list(map(float, argv[3:]))
assert(len(params) == 7)

period = 0.4
traj = pointsTrajectory([[params[1],params[2]]])
leg = Leg(1,1)
controller = myController(params[0], leg, traj, period, [params[3],params[4],params[5],params[6]], 8., 0.2, 3 * np.ones((8, 1)))


# Functions to initialize the simulation and retrieve joints positions/velocities
from .initialization_simulation import configure_simulation, getPosVelJoints

####################
#  INITIALIZATION ##
####################

dt = 0.001  # time step of the simulation
# If True then we will sleep in the main loop to have a 1:1 ratio of (elapsed real time / elapsed time in the
# simulation))
realTimeSimulation = (argv[1] == "True")
enableGUI = (argv[2] == "True")  # enable PyBullet GUI or not
robotId, solo, revoluteJointIndices = configure_simulation(dt, enableGUI)

# Grading
grade = 0

###############
#  MAIN LOOP ##
###############
total_len = 10000
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

    grade -= (qdot[1][0]**2 + qdot[2][0]**2 + qdot[3][0]**2 + qdot[4][0]**2 + qdot[5][0]**2) * (dt**2)
    if(i == total_len-1):
        grade += q[0][0] * (-1./10)

    # Sleep to get a real time simulation
    if realTimeSimulation:
        t_sleep = dt - (time.clock() - t0)
        if t_sleep > 0:
            time.sleep(t_sleep)

print("Result :")
print(str(grade))
# Shut down the PyBullet client
p.disconnect()
