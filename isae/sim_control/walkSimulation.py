# coding: utf8

# Wrapper class for pybullet simulation with adapted parameters for Solo and our controllers

import time

# PyBullet simulator
import pybullet as p 
# Numpy library
import numpy as np  

# footTrajController class
from isae.control.footTrajController import * 
# Leg class
from isae.tools.geometry import * 
# footTrajectory class
from isae.tools.footTrajectory import * 
# for 3D plots in matplotlib
from mpl_toolkits.mplot3d import Axes3D 

# Functions to initialize the simulation and retrieve joints positions/velocities
from isae.sim_control.initialization_simulation_pybullet import configure_simulation, getPosVelJoints

class walkSimulation:
    def __init__(self):

        # Parameters for the main pybullet loop
        # Current simulation step
        self.step = 0
        # Timestep of the simulation
        self.dt = 0.001
        # Total duration of the simulation
        self.duration = 10.
        # Targeted real-time factor (to keep? to delete?)
        self.RTF = 0.
        # pybullet graphical interface
        self.enableGUI = False

        # Trajectories-related parameters

        # Period of the movement 
        self.period = 1.
        # Array of 4 feet trajectories
        self.legsTraj = [] 
        # Body heights wrt. the traj. reference frame during the movement, array of size 4 with values in [0,2[ (unit: leg height)
        self.bodyHeights = [] 

        # Geometry of the leg
        self.Leg = None
        # Kp gain of the controller
        self.Kp = 0. # 8.
        # Kd gain of the controller
        self.Kd = 0. # 0.2
        # Saturation of the controller
        self.sat = float('inf')  # 3 

        # Results to be stored
        # Contacts between the robot and the ground
        self.contactPoints = []
        self.storeContactPoints = True
        # Feet positions in time
        self.feetPos = []
        self.storeFeetPos = True    
        # Base state 
        self.baseState = []
        self.storeBaseState = True

    ###
    # Setting attributes (simulation parameters)
    ###
    def setLoopParams(self,enableGUI, duration, RTF=0.):
        self.enableGUI = enableGUI
        self.duration = duration
        self.RTF = RTF

    def setControllerParams(self,Leg, Kp = 0, Kd = 0, sat = float('inf')):
        self.Leg = Leg
        self.Kp = Kp*1. # 8.
        self.Kd = Kd*1. # 0.2
        self.sat = float('inf') # 3
        
    def setTrajectoryParams(self, period, legsTraj, bodyHeights):
        self.period = period
        self.legsTraj = legsTraj
        self.bodyHeights = bodyHeights

    ###
    # Running the simulation
    ###
    def initializeSim(self):
        robotId, revoluteJointIndices = configure_simulation(self.dt, self.enableGUI) # initializes PyBullet client
        robotController = footTrajController(self.bodyHeights, self.Leg, self.legsTraj, self.period, self.Kp, self.Kd, 3 * np.ones((8, 1)))
        return robotController, robotId, revoluteJointIndices

    def stepSim(self, robotController, robotId, revoluteJointIndices):
        # Time at the start of the loop
        #t0 = time.clock()
        # Get position and velocity of all joints in PyBullet (free flying base + motors)
        q, qdot = getPosVelJoints(robotId, revoluteJointIndices)

        # Call controller to get torques for all joints
        jointTorques = robotController.c(q, qdot, self.step*self.dt, self.dt) # c_walking(q, qdot, dt, solo, i * dt)

        # Set control torques for all joints in PyBullet
        p.setJointMotorControlArray(robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)

        # Compute one step of simulation
        p.stepSimulation()

        # Duration of the step
        #t_step = time.clock() - t0

        if self.storeFeetPos:
            q_joints = q[7:]
            footpos0 = self.Leg.getFootPos(q_joints[0:2])
            footpos1 = self.Leg.getFootPos(q_joints[2:4])
            footpos2 = self.Leg.getFootPos(q_joints[4:6])
            footpos3 = self.Leg.getFootPos(q_joints[6:])

            self.feetPos.append([footpos0,footpos1,footpos2,footpos3])
        
        if self.storeBaseState:
            q_base = q[:7]
            self.baseState.append(q_base)

        if self.storeContactPoints:
            # Store contact points at this time step
            contacts = p.getContactPoints(bodyA = 0) # checks collisions with bodyA=0 (ground plane)
            for point in contacts : 
                self.contactPoints.append([point[5][0], point[5][1], point[4]]) # stores the x,y position of the contact point and the link (index) in contact with the ground

        self.step += 1


    def runSim(self):
        init_sim_date = time.clock()

        # Init
        robotController, robotId, revoluteJointIndices = self.initializeSim()

        # Main loop
        iterations = int(self.duration/self.dt)
        for k in range(iterations):
            #t0 = time.clock()
            self.stepSim(robotController, robotId, revoluteJointIndices)

        # Shut down the PyBullet client
        p.disconnect()

        end_sim_date = time.clock()
        print("##########################################\n"+
            "Simulation finished in " +
            str(end_sim_date - init_sim_date) + " s\n##########################################")

    ###
    # Plotting the results
    ###

    def plotContactPoints(self):
        self.contactPoints = np.array(self.contactPoints)
        colors = self.contactPoints[:,2]

        #fig = plt.figure()
        #ax = fig.add_subplot(1, 1, 1, axisbg="1.0")

        plt.scatter([p[0] for p in self.contactPoints], [p[1] for p in self.contactPoints], c=colors, marker='+')
        plt.title("Contact points with ground")

    def plotFeetAvgPos(self):
        self.feetPos = np.array(self.feetPos)
        
        #0.19 0.1046

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        plt.plot(self.feetPos[:,0,0] + 0.19, [0.1046]*len(self.feetPos[:,0,0]), self.feetPos[:,0,1])
        plt.plot(self.feetPos[:,1,0] + 0.19, [-0.1046]*len(self.feetPos[:,0,0]), self.feetPos[:,1,1])
        plt.plot(self.feetPos[:,2,0] - 0.19, [0.1046]*len(self.feetPos[:,0,0]), self.feetPos[:,2,1])
        plt.plot(self.feetPos[:,3,0] - 0.19, [-0.1046]*len(self.feetPos[:,0,0]), self.feetPos[:,3,1])
        plt.show()

        # Plot feet trajectories
        plt.title("Feet average cycle")
    
    def plotBasePos(self):
        self.baseState = np.array(self.baseState)

        #fig = plt.figure()
        #ax = fig.add_subplot(1, 1, 1, axisbg="1.0")

        plt.plot(self.baseState[:,0], self.baseState[:,1])