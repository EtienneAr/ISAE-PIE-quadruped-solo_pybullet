# coding: utf8

#####################
#  LOADING MODULES ##
#####################

import time

import pybullet as p  # PyBullet simulator
import numpy as np  # Numpy library

from sys import argv

from isae.control.footTrajController import *
from isae.tools.geometry import *
from isae.tools.footTrajectory import *
import isae.optim.grading
from mpl_toolkits.mplot3d import Axes3D

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

grading = isae.optim.grading.grading_RMS()

# Functions to initialize the simulation and retrieve joints positions/velocities
from .initialization_simulation import configure_simulation, getPosVelJoints

####################
#  INITIALIZATION ##
####################
RTF = 1 # Real Time Factor
dt = 0.001  # time step of the simulation

# If True then we will sleep in the main loop to have a 1:1 ratio of (elapsed real time / elapsed time in the
# simulation))
realTimeSimulation = (argv[2] == "True")
enableGUI = (argv[1] == "True")  # enable PyBullet GUI or not
robotId, solo, revoluteJointIndices = configure_simulation(dt, enableGUI)

# Feet trajectories
contTraj1 = footTrajectory([[-0.7,0],[-0.2,0.6], [0.3,0], [-0.7,0]], phaseOffset = offsets[0])
contTraj2 = footTrajectory(contTraj1.points, phaseOffset = offsets[1])
contTraj3 = footTrajectory([[-0.3,0],[0.2,1], [0.7,0], [-0.3,0]], phaseOffset = offsets[2])
contTraj4 = footTrajectory(contTraj3.points, phaseOffset = offsets[3])

trajs = [contTraj1, contTraj2, contTraj3, contTraj4]

# Geometry and controller
leg = Leg(1,1)
controller = footTrajController(bodyHeight, leg, trajs, period, Kp, Kd, 3 * np.ones((8, 1)))

# Grading
goal_factors = np.vstack([75, 10, 10, 1, 1, 1])
goal_speed = np.vstack([.5, 0, 0, 0, 0, 0])

# Plotting
leg0_jointsPos = []
leg1_jointsPos = []
leg2_jointsPos = []
leg3_jointsPos = []
contact_points = []

###############
#  MAIN LOOP ##
###############

total_duration = 10.
total_len = int(total_duration/dt)

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
    
    # Store contact points at this time step
    contacts = p.getContactPoints(bodyA = 0)
    for point in contacts : 
        contact_points.append([point[5][0], point[5][1], point[4]])



leg0_footpos = map(lambda joints_pos : leg.getFootPos(joints_pos), leg0_jointsPos)
leg1_footpos = map(lambda joints_pos : leg.getFootPos(joints_pos), leg1_jointsPos)
leg2_footpos = map(lambda joints_pos : leg.getFootPos(joints_pos), leg2_jointsPos)
leg3_footpos = map(lambda joints_pos : leg.getFootPos(joints_pos), leg3_jointsPos)

#print grading
print("Result :")
print(str(grading.getGrade()))

# Shut down the PyBullet client
p.disconnect()

# Plot feet trajectories
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

plt.plot([p[0] + 0.5 for p in leg0_footpos], [0.4]*len(leg0_footpos), [p[1] for p in leg0_footpos])
plt.plot([p[0] + 0.5 for p in leg1_footpos], [-0.4]*len(leg0_footpos), [p[1] for p in leg1_footpos])
plt.plot([p[0] - 0.5 for p in leg2_footpos], [0.4]*len(leg0_footpos), [p[1] for p in leg2_footpos])
plt.plot([p[0] - 0.5 for p in leg3_footpos], [-0.4]*len(leg0_footpos), [p[1] for p in leg3_footpos])
plt.title("Feet trajectories")

# Plot robot contacts with ground
contact_points = np.array(contact_points)
colors = contact_points[:,2]

fig2 = plt.figure()
plt.scatter([p[0] for p in contact_points], [p[1] for p in contact_points], c=colors, marker='+')
plt.title("Contact points with ground")

plt.show()
