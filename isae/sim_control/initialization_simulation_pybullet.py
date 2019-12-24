# coding: utf8

import numpy as np  # Numpy library

import pybullet as p  # PyBullet simulator
import pybullet_data
import pybullet_utils.bullet_client as bc
#from example_robot_data import loadSolo  # Functions to load the SOLO quadruped


def configure_simulation(dt, enableGUI):
    global jointTorques

    # Load the robot for Pinocchio

    #solo = loadSolo(True) # load solo from URDF files for gepetto-gui. If false, load solo12
    #solo.initDisplay(loadModel=True) # interacts with gepetto-gui, MUCH FASTER EXECUTION WHEN COMMENTED OUT

    # Start the client for PyBullet
    if enableGUI:
        physicsClient = bc.BulletClient(connection_mode=p.GUI)
    else:
        physicsClient = bc.BulletClient(connection_mode=p.DIRECT)
        #physicsClient = p.connect(p.DIRECT)  # noqa
    # p.GUI for graphical version
    # p.DIRECT for non-graphical version

    # Set gravity (disabled by default)
    physicsClient.setGravity(0, 0, -9.81)

    # Load horizontal plane for PyBullet
    physicsClient.setAdditionalSearchPath(pybullet_data.getDataPath())
    groundPlaneId = physicsClient.loadURDF("plane.urdf")

    # Set friction coeff of ground plane
    physicsClient.changeDynamics(groundPlaneId, -1, lateralFriction=2)

    # Load the robot for PyBullet
    robotStartPos = [0, 0, 0.35]
    robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
    physicsClient.setAdditionalSearchPath("./solo_description/robots")
    robotId = physicsClient.loadURDF("solo.urdf", robotStartPos, robotStartOrientation)

    # Set time step of the simulation
    # dt = 0.001
    physicsClient.setTimeStep(dt)
    # realTimeSimulation = True # If True then we will sleep in the main loop to have a frequency of 1/dt

    # Disable default motor control for revolute joints
    revoluteJointIndices = [0, 1, 3, 4, 6, 7, 9, 10]
    physicsClient.setJointMotorControlArray(robotId,
                                jointIndices=revoluteJointIndices,
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocities=[0.0 for m in revoluteJointIndices],
                                forces=[0.0 for m in revoluteJointIndices])

    # Enable torque control for revolute joints
    jointTorques = [0.0 for m in revoluteJointIndices]
    physicsClient.setJointMotorControlArray(robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)

    # Compute one step of simulation for initialization
    physicsClient.stepSimulation()

    # return robotId, solo, revoluteJointIndices, physicsClient
    return robotId, revoluteJointIndices, physicsClient


# Function to get the position/velocity of the base and the angular position/velocity of all joints
def getPosVelJoints(robotId, revoluteJointIndices, physicsClient):

    jointStates = physicsClient.getJointStates(robotId, revoluteJointIndices)  # State of all joints
    baseState = physicsClient.getBasePositionAndOrientation(robotId)  # Position of the free flying base
    baseVel = physicsClient.getBaseVelocity(robotId)  # Velocity of the free flying base

    # Reshaping data into q and qdot
    q = np.vstack((np.array([baseState[0]]).transpose(), np.array([baseState[1]]).transpose(),
                   np.array([[jointStates[i_joint][0] for i_joint in range(len(jointStates))]]).transpose()))
    qdot = np.vstack((np.array([baseVel[0]]).transpose(), np.array([baseVel[1]]).transpose(),
                      np.array([[jointStates[i_joint][1] for i_joint in range(len(jointStates))]]).transpose()))

    return q, qdot