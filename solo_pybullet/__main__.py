# coding: utf8

#####################
#  LOADING MODULES ##
#####################

import time

import pybullet as p  # PyBullet simulator
import numpy as np  # Numpy library

###############################
##  Custom parameter input   ##
###############################
from sys import argv

fractorTraj, pointsTraj, period, bodyHeight, Kp, Kd, offsets = (None,) * 7

params = list(map(float, argv[3:]))
try:
    realTimeSimulation = (argv[2] == "True")
    enableGUI = (argv[1] == "True")  # enable PyBullet GUI or not
    
    bodyHeight = params[0]
    period = params[1]
    fractorTraj = [params[2], 1]
    offsets = [params[3],params[4],params[5],params[6]]
    pointsTraj = [[params[7],params[8]], [params[9],params[10]], [params[11],params[12]]]
    if(len(params) == 15):
        Kp = float(params[-2])
        Kd = float(params[-1])
    else:
        Kp = 8.0
        Kd = 0.2
        assert(len(params) == 13)
    # for i in range(9, len(params), 2):
    #     pointsTraj += [[params[i], params[i+1]]]
except:
    print(" # Error. Espected parameters are : ")
    print(" # # guiOn rtSimuOn bodyHeight stepPeriod stepLen phaseOffset_1 phaseOffset_2 phaseOffset_3 phaseOffset_4 point0_X point0_Y point1_X point1_Y point2_X point2_Y [Kp Kd]")
    quit()

##############################
##  Define custom objects   ##
##############################
import isae.control.myController
import isae.tools.geometry
import isae.tools.trajectory
import isae.optim.grading

controller = isae.control.myController.myController(
    bodyHeight, #Goal body height
    isae.tools.geometry.Leg(1,1), #geometry
    isae.tools.trajectory.pointsTrajectory(pointsTraj, factor=fractorTraj), #trajectoryGenerator
    period,     #period
    offsets,    #phase Offsets
    Kp,         #proportionnal gain
    Kd,         #derivative gaine
    3 * np.ones((8, 1)) #torque limits
    ) 

grading = isae.optim.grading.grading_RMS(
    np.vstack([.5, 0, 0, 0, 0, 0]), # qdot_ref
    np.vstack([75, 10, 10, 1, 1, 1]) # factors
    )

####################
#  INITIALIZATION ##
####################
# Functions to initialize the simulation and retrieve joints positions/velocities
from .initialization_simulation import configure_simulation, getPosVelJoints

dt = 0.001  # time step of the simulation
# If True then we will sleep in the main loop to have a 1:1 ratio of (elapsed real time / elapsed time in the
# simulation))
robotId, revoluteJointIndices = configure_simulation(dt, enableGUI)

###############
#  MAIN LOOP ##
###############
total_duration = 10. #s
total_len = int(total_duration / dt)
for i in range(total_len):  # run the simulation during dt * i_max seconds (simulation time)

    # Time at the start of the loop
    if realTimeSimulation:
        t0 = time.clock()

    # Get position and velocity of all joints in PyBullet (free flying base + motors)
    q, qdot = getPosVelJoints(robotId, revoluteJointIndices)

    # Call controller to get torques for all joints
    jointTorques = controller.c(q, qdot, i*dt, dt)

    # Set control torques for all joints in PyBullet
    p.setJointMotorControlArray(robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)

    # Compute one step of simulation
    p.stepSimulation()

    grading.updateGrade(q, qdot, i*dt, dt)

    # Sleep to get a real time simulation
    if realTimeSimulation:
        t_sleep = dt - (time.clock() - t0)
        if t_sleep > 0:
            time.sleep(t_sleep)

#print grading
print("Result :")
print(str(grading.getGrade()))

# Shut down the PyBullet client
p.disconnect()
