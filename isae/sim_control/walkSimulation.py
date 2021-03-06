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
# grading class : in optim module, expects grade and getGrade (aka final grade) as class methods

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
        # IK solution to be used, array of boolean of size 4 (see the Geometry class)
        #self.sols = []
        # Kp gain of the controller
        #self.Kp = 0. # 8.
        # Kd gain of the controller
        #self.Kd = 0. # 0.2
        # Saturation of the controller
        #self.sat = float('inf')  # 3 

        # Results to be stored
        # Contacts between the robot and the ground
        self.contactPoints = []
        self.storeContactPoints = True
        # Feet positions in time
        self.feetPos = []
        self.storeFeetPos = True    
        # Base state 
        self.qBase = []
        self.qdotBase = []
        self.store_qBase = True

        # Internal variables
        self.robotController = None 
        self.robotId = None
        self.revoluteJointIndices = None

    ###
    # Setting attributes (simulation parameters)
    ###
    def setLoopParams(self,enableGUI, duration, RTF=0.):
        self.enableGUI = enableGUI
        self.duration = duration
        self.RTF = RTF

    #def setControllerParams(self,Leg, sols = 4*[False], Kp = 0, Kd = 0, sat = float('inf')):
    #    self.Leg = Leg
    #    self.sols = sols
    #    self.Kp = Kp*1. # 8.
    #    self.Kd = Kd*1. # 0.2
    #    self.sat = float('inf') # 3

    def setController(self, robotController):
        self.robotController = robotController
        
    def setTrajectoryParams(self, leg, period, legsTraj, bodyHeights):
        self.Leg = leg
        self.period = period
        self.legsTraj = legsTraj
        self.bodyHeights = bodyHeights

    ###
    # Running the simulation
    ###
    def initializeSim(self):
        self.robotId, self.revoluteJointIndices = configure_simulation(self.dt, self.enableGUI) # initializes PyBullet client
        #self.robotController = footTrajController(self.bodyHeights, self.Leg, self.sols, self.legsTraj, self.period, self.Kp, self.Kd, 3 * np.ones((8, 1)))

    def stepSim(self):
        # Time at the start of the loop
        #t0 = time.clock()
        # Get position and velocity of all joints in PyBullet (free flying base + motors)
        q, qdot = getPosVelJoints(self.robotId, self.revoluteJointIndices)

        # Call controller to get torques for all joints
        jointTorques = self.robotController.c(q, qdot, self.step*self.dt, self.dt) # c_walking(q, qdot, dt, solo, i * dt)

        # Set control torques for all joints in PyBullet
        p.setJointMotorControlArray(self.robotId, self.revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)

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
        
        if self.store_qBase:
            q_base = q[:7]
            qdot_base = qdot[:7]
            self.qBase.append(q_base)
            self.qdotBase.append(qdot_base)

        if self.storeContactPoints:
            # Store contact points at this time step
            contacts = p.getContactPoints(bodyA = 0) # checks collisions with bodyA=0 (ground plane)
            for point in contacts : 
                self.contactPoints.append([point[5][0], point[5][1], point[4]]) # stores the x,y position of the contact point and the link (index) in contact with the ground

        self.step += 1


    def runSim(self):
        init_sim_date = time.clock()

        # Main loop
        iterations = int(self.duration/self.dt)
        for k in range(iterations):
            #t0 = time.clock()
            self.stepSim()

        # Shut down the PyBullet client
        p.disconnect()

        end_sim_date = time.clock()
        print("##########################################\n"+
            "Simulation finished in " +
            str(end_sim_date - init_sim_date) + " s\n##########################################")
    
    ###
    # Helpful functions for grading
    ###
    # Return rolling average of array param with a window of size n
    def rollingAvg(self, param, n):
        arr = np.array(param)
        rolling_avg = np.zeros(arr.shape)
        l = len(arr)
        for k in range(n):
            rolling_avg = rolling_avg + np.concatenate((k*[0.],arr[:l-k]))
        rolling_avg /= n
        return rolling_avg
    
    # Return distance between the initial base position (x0,y0) 
    # and the final base position (xf,yf)
    # offset : number of steps to skip before setting (x0,y0)
    def getFinalDistance(self, offset=0):
        self.qBase = np.array(self.qBase)
        x0, y0 = self.qBase[:,0][0+offset], self.qBase[:,1][0+offset]
        xf, yf = self.qBase[:,0][-1], self.qBase[:,1][-1]

        return np.sqrt((xf-x0)**2 + (yf-y0)**2)[0]
    ###
    # Plotting the results
    ###

    def plotContactPoints(self):
        self.contactPoints = np.array(self.contactPoints)
        colors = self.contactPoints[:,2]

        plt.scatter(self.contactPoints[:,0], self.contactPoints[:,1], c=colors, marker='+',s=25)
        plt.title("Contact points with ground")

    def plotFeetAvgPos(self):
        self.feetPos = np.array(self.feetPos)
        
        #0.19 0.1046

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        plt.plot(self.feetPos[:,0,0] + 0.19, [0.1046]*len(self.feetPos[:,0,0]), self.feetPos[:,0,1], label='Foot 0 traj.')
        plt.plot(self.feetPos[:,1,0] + 0.19, [-0.1046]*len(self.feetPos[:,0,0]), self.feetPos[:,1,1], label='Foot 1 traj.')
        plt.plot(self.feetPos[:,2,0] - 0.19, [0.1046]*len(self.feetPos[:,0,0]), self.feetPos[:,2,1], label='Foot 2 traj.')
        plt.plot(self.feetPos[:,3,0] - 0.19, [-0.1046]*len(self.feetPos[:,0,0]), self.feetPos[:,3,1], label='Foot 3 traj.')
        plt.legend()
        plt.show()

        # Plot feet trajectories
        #plt.title("Feet average cycle")
    
    def plotBasePos(self):
        self.qBase = np.array(self.qBase)

        #fig = plt.figure()
        #ax = fig.add_subplot(1, 1, 1, axisbg="1.0")

        x0, y0 = self.qBase[:,0][0], self.qBase[:,1][0]
        xf, yf = self.qBase[:,0][-1], self.qBase[:,1][-1]
        
        filter_size = 1000
        xAvg = self.rollingAvg(self.qBase[:,0,0], filter_size)
        yAvg = self.rollingAvg(self.qBase[:,1,0], filter_size)

        plt.plot(self.qBase[:,0], self.qBase[:,1], label='Base xy traj.')
        plt.plot(xAvg, yAvg, 'r', linewidth=2, label='Avg. base xy traj.')
        plt.plot([x0,xf],[y0,yf], '--', linewidth = 2)
    
    def plotBaseSpeed(self):
        self.qdotBase = np.array(self.qdotBase)

        filter_size = 500
        xdotAvg = self.rollingAvg(self.qdotBase[:,0,0], filter_size)
        ydotAvg = self.rollingAvg(self.qdotBase[:,1,0], filter_size)
        #fig = plt.figure()
        #ax = fig.add_subplot(1, 1, 1, axisbg="1.0")

        #xdot0, y0 = self.qBase[:,0][0], self.qBase[:,1][0]
        #xf, yf = self.qBase[:,0][-1], self.qBase[:,1][-1]

        plt.plot(self.qdotBase[:,0,0],self.qdotBase[:,1,0], 'g',linewidth=0.5, label='Base xy speed')
        plt.plot(xdotAvg,ydotAvg, 'r', linewidth=2,label='Avg base speed')
        #plt.plot([x0,xf],[y0,yf], '--', linewidth = 2)

    def plotTrajBase(self):
        fig = plt.figure()
        #ax = fig.add_subplot(1, 1, 1, axisbg="1.0")
        ax = fig.add_subplot(2,1,1)
        ax.set_xlabel('x_world (m)')
        ax.set_ylabel('y_world (m)')
        self.plotContactPoints()
        self.plotBasePos()

        ax2 = fig.add_subplot(2,1,2)
        self.feetPos = np.array(self.feetPos)
        x_foot_0 = self.qBase[:,0,0] + self.feetPos[:,0,0]
        z_foot_0 = self.qBase[:,2,0] + self.feetPos[:,0,1]
        plt.plot(self.qBase[:,0,0],self.qBase[:,2,0], '-', linewidth=2, label='Base xz traj.')
        plt.plot(x_foot_0,z_foot_0, '-', label='Foot traj. + base mvt.')

        ax.legend()
        ax.grid(True)
        ax2.legend()
        ax2.grid(True)
        plt.show()