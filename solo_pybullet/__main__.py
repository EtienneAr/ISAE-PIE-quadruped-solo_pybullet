# coding: utf8

#####################
#  LOADING MODULES ##
#####################

import time

import pybullet as p  # PyBullet simulator
import numpy as np  # Numpy library
<<<<<<< Updated upstream
=======
import matplotlib.pyplot as plt
<<<<<<< Updated upstream
from mpl_toolkits.mplot3d import Axes3D
=======
>>>>>>> Stashed changes
>>>>>>> Stashed changes

###############################
##  Custom parameter input   ##
###############################
from sys import argv

fractorTraj, pointsTraj, period, bodyHeight, Kp, Kd, offsets = (None,) * 7

params = list(map(float, argv[3:]))
try:
<<<<<<< Updated upstream
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
<<<<<<< Updated upstream
        assert(len(params) == 13)
    # for i in range(9, len(params), 2):
    #     pointsTraj += [[params[i], params[i+1]]]
=======
        assert(len(params) == 6 + nb_point_traj)


=======
    if(len(params) > 16):
        realTimeSimulation = (argv[2] == "True")
        enableGUI = (argv[1] == "True")  # enable PyBullet GUI or not

        bodyHeight = params[0]
        period = params[1]
        fractorTraj = [params[2], 1]
        offsets = [params[3],params[4],params[5],params[6]]
        pointsTraj = [[params[7],params[8],params[9],params[10],params[11],params[12],params[13],params[14]],[params[15],params[16],params[17],params[18],params[19],params[20],params[21],params[22]]]
        Kp = params[23]
        Kd = params[24]
                      
    else:
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
>>>>>>> Stashed changes
>>>>>>> Stashed changes
except:
    print(" # Error. Espected parameters are : ")
    print(" # # guiOn rtSimuOn bodyHeight stepPeriod stepLen phaseOffset_1 phaseOffset_2 phaseOffset_3 phaseOffset_4 point0_X point0_Y point1_X point1_Y point2_X point2_Y [Kp Kd]")
    quit()

##############################
##  Define custom objects   ##
##############################
import isae.control.myController
import isae.tools.geometry
<<<<<<< Updated upstream
=======
import isae.tools.trajectory_TC
<<<<<<< Updated upstream
>>>>>>> Stashed changes
import isae.tools.trajectory
=======
>>>>>>> Stashed changes
import isae.optim.grading
from isae.tools.cameraTool import *

#pointsTraj = [[-0.35,0.0,0.35,0.0,0.25,0.25,-0.25,0.25],[0.1,0, 0.1, 0.0, -0.1, 0.1, -0.1, -0.1]] 
#pointsTraj = [[-0.3625019854839524, 0.0, 0.3680379448472392, 0.0, 0.20191187979355665, 0.2846827727467334, -0.21830796075142933, 0.5634651856469657], [0.17336827624885176, 0.0, 0.013602855337730224, 0.0, -0.10185494005696819, 0.12554799594359525, -0.20082772637661345, -0.18007033604140782]]

controller = isae.control.myController.myController(
    bodyHeight, #Goal body height
    isae.tools.geometry.Leg(1,1), #geometry
<<<<<<< Updated upstream
    isae.tools.trajectory.pointsTrajectory(pointsTraj, factor=fractorTraj), #trajectoryGenerator
=======
<<<<<<< Updated upstream
    # use the trajectory class corresponding to pointsTraj
    isae.tools.trajectory_TC.pointsTrajectory(pointsTraj, factor=fractorTraj), #trajectoryGenerator
=======
    isae.tools.trajectory_TC.pointsTrajectory(pointsTraj, factor=fractorTraj ), #trajectoryGenerator
>>>>>>> Stashed changes
>>>>>>> Stashed changes
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

#################
# Start saving .png image
#################
nb_image = 0
#saveImage(getRGBAImage(),nb_image)
len_images = 24*3


###############
#  MAIN LOOP ##
###############
<<<<<<< Updated upstream
total_duration = 10. #s
=======
<<<<<<< Updated upstream
total_duration = 5 #s
=======
total_duration = 5. #s
>>>>>>> Stashed changes
>>>>>>> Stashed changes
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

<<<<<<< Updated upstream
=======
<<<<<<< Updated upstream
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
=======
    if i > 100:        
        if i%int(total_len/len_images) == 0:
            nb_image = nb_image + 1
            saveImage(getRGBAImage(),nb_image)
    

>>>>>>> Stashed changes
>>>>>>> Stashed changes
#print grading
print("Result :")
print(str(grading.getGrade()))

# Shut down the PyBullet client
p.disconnect()
<<<<<<< Updated upstream
=======
<<<<<<< Updated upstream
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
=======

#saveVideo()

#plt.imshow(rgb_opengl)
#plt.show()
>>>>>>> Stashed changes
>>>>>>> Stashed changes
