# coding: utf8

#####################
#  LOADING MODULES ##
#####################

import time

import pybullet as p  # PyBullet simulator
import numpy as np  # Numpy library
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

###############################
##  Custom parmaters input   ##
###############################
from sys import argv
start_time = time.time()
fractorTraj, pointsTraj, period, bodyHeight, Kp, Kd, offsets = (None,) * 7

params = list(map(float, argv[4:]))

try:
    realTimeSimulation = (argv[2] == "True")
    enableGUI = (argv[1] == "True")  # enable PyBullet GUI or not
    nb_point_traj = int(argv[3]) #nombre de points (1 point = 2 arguments)
    
    bodyHeight = params[0]
    period = params[1]
    fractorTraj = [params[2], 1]
    offsets = [params[3],params[4],params[5],params[6]]

    

    # try to introduce more traj points as argument, 
    # to keep the same main file for all kind of trajectories 
    pointsTraj = []
    for k in range(nb_point_traj):
        pointsTraj.append([params[6+k*2+1] , params[6+k*2+2] ])
    if(len(params) == (7 + nb_point_traj*2 + 2)):
        Kp = float(params[-2])
        Kd = float(params[-1])
    else:
        Kp = 8.0
        Kd = 0.2
        assert(len(params) == 6 + nb_point_traj)


except:
    print(" # Error. Espected parameters are : ")
    print(" # # guiOn rtSimuOn bodyHeight stepPeriod stepLen phaseOffset_1 phaseOffset_2 phaseOffset_3 phaseOffset_4 point0_X point0_Y point1_X point1_Y point2_X point2_Y [Kp Kd]")
    quit()

print(pointsTraj)

##############################
##  Define custom objects   ##
##############################
import isae.control.myController
import isae.tools.geometry
import isae.tools.trajectory_TC
import isae.tools.trajectory
import isae.optim.grading


#Test trajectory_TC, bezier curves
#pointsTraj =[[-0.25, 0.25], [0.16909430327524033, 0.6149866435075454], [-0.11340611589329358, 0.39965743876613913], [0.25, 0.7], [0.6134061158932935, 1.0003425612338608], [0.5671101844243762, 0.0], [0.3, 0.0], [0.03288981557562376, 0.0], [0.0732682754563333, 0.0], [-0.3, 0.0], [-0.6732682754563333, 0.0], [-0.6690943032752403, -0.1149866435075454], [-0.25, 0.25]]


controller = isae.control.myController.myController(
    bodyHeight, #Goal body height
    isae.tools.geometry.Leg(1,1), #geometry
    # use the trajectory class corresponding to pointsTraj
    isae.tools.trajectory_TC.pointsTrajectory(pointsTraj, factor=fractorTraj), #trajectoryGenerator
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


# Plotting
# leg0_jointsPos = []
# leg1_jointsPos = []
# leg2_jointsPos = []
# leg3_jointsPos = []
# contact_points = []

# Geometry and controller
leg = isae.tools.geometry.Leg(1,1)


####################
#  INITIALIZATION ##
####################
# Functions to initialize the simulation and retrieve joints positions/velocities
from .initialization_simulation import configure_simulation, getPosVelJoints

dt = 0.001  # time step of the simulation
# If True then we will sleep in the main loop to have a 1:1 ratio of (elapsed real time / elapsed time in the
# simulation))
robotId, solo, revoluteJointIndices = configure_simulation(dt, enableGUI)

###############
#  MAIN LOOP ##
###############
total_duration = 5 #s
total_len = int(total_duration / dt)
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

    grading.updateGrade(q, qdot, i*dt, dt)

    # Sleep to get a real time simulation
    if realTimeSimulation:
        t_sleep = dt - (time.clock() - t0)
        if t_sleep > 0:
            time.sleep(t_sleep)

    # Store leg1 joint values
    # leg0_jointsPos.append([q[7], q[8]])
    # leg1_jointsPos.append([q[9], q[10]])
    # leg2_jointsPos.append([q[11], q[12]])
    # leg3_jointsPos.append([q[13], q[14]])

    # contacts = p.getContactPoints(bodyA = 0)
    # for point in contacts : 
    #     contact_points.append([point[5][0], point[5][1], point[4]])


# leg0_footpos = map(lambda joints_pos : leg.getFootPos(joints_pos), leg0_jointsPos)
# leg1_footpos = map(lambda joints_pos : leg.getFootPos(joints_pos), leg1_jointsPos)
# leg2_footpos = map(lambda joints_pos : leg.getFootPos(joints_pos), leg2_jointsPos)
# leg3_footpos = map(lambda joints_pos : leg.getFootPos(joints_pos), leg3_jointsPos)
#print grading
print("Result :")
print(str(grading.getGrade()))

# Shut down the PyBullet client
p.disconnect()
print("Temps d execution : %s secondes ---" % (time.time() - start_time))
#Plot feet trajectories
# fig = plt.figure()

# a = []
# b = [] 

# for elt in leg0_footpos:
#     a.append(elt[0] )
#     b.append(elt[1] + 1)

# plt.plot(a,b)

# T = np.linspace(0,1,500)
# X = []
# Y = []

# FootTraj = isae.tools.trajectory_TC.pointsTrajectory(pointsTraj, factor=fractorTraj)
# for elt in T:
#     X.append(FootTraj.getPos(elt, None)[0])
#     Y.append(FootTraj.getPos(elt, None)[1])

# fig = plt.figure(2)
# plt.plot(X , Y,'x')
# plt.show()